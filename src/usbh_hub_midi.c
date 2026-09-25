/* usbh_hub_midi -- see usbh_hub_midi.h. */
#include "usbh_hub_midi.h"

#include <string.h>

#include "stm32h7xx_hal.h"
#include "usbh_ctlreq.h"
#include "usbh_ioreq.h"
#include "usbh_pipes.h"

#define DMA_SECTION __attribute__((section(".sram1_bss")))

#define HUB_CLASS 0x09U
#define DEV_ADDR0 0x02U /* first device address; the hub keeps USBH_DEVICE_ADDRESS (1) */

/* USB 2.0 chapter 11. */
#define HUB_REQ_GET_STATUS    0x00U
#define HUB_REQ_CLEAR_FEATURE 0x01U
#define HUB_REQ_SET_FEATURE   0x03U
#define HUB_DESC_TYPE         0x29U
#define PORT_ENABLE           1U
#define PORT_RESET            4U
#define PORT_POWER            8U
#define C_PORT_CONNECTION     16U
#define C_PORT_RESET          20U
#define PS_CONNECTION         0x0001U
#define PS_ENABLE             0x0002U
#define PS_RESET              0x0010U
#define PS_LOW_SPEED          0x0200U

enum
{
    S_HUB_DESC = 1,
    S_POWER,
    S_POWER_WAIT,
    S_SCAN,
    S_SCAN_WAIT,
    S_CLR_C_CONN,
    S_RESET,
    S_RESET_POLL,
    S_CLR_C_RESET,
    S_RECOVERY,
    S_DEV_DESC8,
    S_DEV_ADDR,
    S_DEV_ADDR_WAIT,
    S_DEV_DESC18,
    S_DEV_CFG9,
    S_DEV_CFG_FULL,
    S_DEV_SET_CFG,
    S_RX,
    S_RX_POLL,
    S_FAILED,
    S_REJECT,     /* the device is not MIDI: switch its port off, scan on */
};

typedef struct
{
    uint8_t  state;
    uint8_t  ports;
    uint16_t pwr_wait_ms;
    uint8_t  port;      /* port being scanned / used, 1-based */
    uint16_t skip;      /* ports whose device was not MIDI (bit n = port n) */
    uint8_t  dev_addr;  /* address for the device being enumerated */
    uint8_t  hub_mps;   /* the hub's control max packet size */
    uint8_t  rejects;
    uint32_t t0;
    uint16_t port_status, port_change;
    uint8_t  dev_mps0;
    uint16_t cfg_total;
    uint8_t  cfg_value;
    uint8_t  in_ep, out_ep, in_pipe, out_pipe;
    uint16_t in_size, out_size;
    uint8_t  ready;
    uint16_t vid, pid;
    uint8_t  fail_state, fail_code;
} HubMidi;

static HubMidi            H;
static HUBMIDI_RxCallback s_cb;
static void              *s_user;

/* Control-transfer data and the MIDI receive buffer: DMA-reachable. */
static uint8_t DMA_SECTION s_ctl[512];
static uint8_t DMA_SECTION s_rx[64];

/* ---------------------------------------------------------------- helpers */

static USBH_StatusTypeDef ctl(USBH_HandleTypeDef *ph, uint8_t type, uint8_t req,
                              uint16_t val, uint16_t idx, uint8_t *buf, uint16_t len)
{
    if (ph->RequestState == CMD_SEND)
    {
        ph->Control.setup.b.bmRequestType = type;
        ph->Control.setup.b.bRequest      = req;
        ph->Control.setup.b.wValue.w      = val;
        ph->Control.setup.b.wIndex.w      = idx;
        ph->Control.setup.b.wLength.w     = len;
    }
    return USBH_CtlReq(ph, buf, len);
}

/* Point the core's control pipes at another device address. */
static void ctl_target(USBH_HandleTypeDef *ph, uint8_t addr, uint8_t mps)
{
    ph->Control.pipe_size = mps;
    USBH_OpenPipe(ph, ph->Control.pipe_in, 0x80U, addr, ph->device.speed, USBH_EP_CONTROL, mps);
    USBH_OpenPipe(ph, ph->Control.pipe_out, 0x00U, addr, ph->device.speed, USBH_EP_CONTROL, mps);
}

static void fail(USBH_StatusTypeDef st)
{
    H.fail_state = H.state;
    H.fail_code  = (uint8_t)st;
    /* A device behind the hub that will not enumerate, or is not MIDI (a
     * dongle's own Ethernet chip, a card reader), costs its port, not the
     * hub: switch the port off and keep scanning. */
    H.state = (H.state >= S_DEV_DESC8 && H.state <= S_DEV_SET_CFG) ? S_REJECT : S_FAILED;
}

/* Step a control request: OK -> next state, BUSY -> stay, else fail. */
static int step(USBH_StatusTypeDef st, uint8_t next)
{
    if (st == USBH_OK)
    {
        H.state = next;
        return 1;
    }
    if (st != USBH_BUSY) fail(st);
    return 0;
}

static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

/* Find the first MIDI-streaming interface's bulk endpoints. */
static int parse_cfg(const uint8_t *c, uint16_t total)
{
    uint16_t i    = 0;
    int      midi = 0;
    H.in_ep = H.out_ep = 0;
    while (i + 2 <= total)
    {
        const uint8_t len = c[i], type = c[i + 1];
        if (len < 2 || i + len > total) break;
        if (type == 0x04 && len >= 9) /* interface */
        {
            if (midi && H.in_ep && H.out_ep) break;
            midi = (c[i + 5] == 0x01 && c[i + 6] == 0x03);
        }
        else if (type == 0x05 && len >= 7 && midi && (c[i + 3] & 0x03) == 0x02)
        {
            const uint8_t  ep  = c[i + 2];
            const uint16_t mps = le16(&c[i + 4]) & 0x03FFU;
            if (ep & 0x80U) { if (!H.in_ep)  { H.in_ep = ep;  H.in_size = mps; } }
            else            { if (!H.out_ep) { H.out_ep = ep; H.out_size = mps; } }
        }
        i = (uint16_t)(i + len);
    }
    if (H.in_size > sizeof s_rx) H.in_size = sizeof s_rx;
    return H.in_ep && H.out_ep;
}

/* ---------------------------------------------------------------- class */

static USBH_StatusTypeDef Init(USBH_HandleTypeDef *ph)
{
    memset(&H, 0, sizeof H);
    H.state                   = S_HUB_DESC;
    H.hub_mps                 = (uint8_t)ph->Control.pipe_size;
    H.dev_addr                = DEV_ADDR0;
    ph->pActiveClass->pData   = &H;
    return USBH_OK;
}

static USBH_StatusTypeDef DeInit(USBH_HandleTypeDef *ph)
{
    if (H.in_pipe)  { USBH_ClosePipe(ph, H.in_pipe);  USBH_FreePipe(ph, H.in_pipe);  }
    if (H.out_pipe) { USBH_ClosePipe(ph, H.out_pipe); USBH_FreePipe(ph, H.out_pipe); }
    H.in_pipe = H.out_pipe = 0;
    H.ready   = 0;
    if (ph->pActiveClass) ph->pActiveClass->pData = 0;
    return USBH_OK;
}

static USBH_StatusTypeDef Requests(USBH_HandleTypeDef *ph)
{
    (void)ph;
    return USBH_OK; /* all the work is in the background process */
}

static USBH_StatusTypeDef Process(USBH_HandleTypeDef *ph)
{
    const uint32_t now = HAL_GetTick();
    switch (H.state)
    {
        case S_HUB_DESC:
            if (step(ctl(ph, USB_D2H | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_DEVICE,
                         USB_REQ_GET_DESCRIPTOR, (uint16_t)(HUB_DESC_TYPE << 8), 0, s_ctl, 9),
                     S_POWER))
            {
                H.ports       = s_ctl[2];
                H.pwr_wait_ms = (uint16_t)(s_ctl[5] * 2U);
                if (H.ports == 0 || H.ports > 15) H.ports = 4;
                H.port = 1;
            }
            break;

        case S_POWER:
            if (step(ctl(ph, USB_H2D | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_OTHER,
                         HUB_REQ_SET_FEATURE, PORT_POWER, H.port, 0, 0),
                     S_POWER))
            {
                if (++H.port > H.ports)
                {
                    H.port  = 1;
                    H.t0    = now;
                    H.state = S_POWER_WAIT;
                }
            }
            break;

        case S_POWER_WAIT:
            if (now - H.t0 >= (uint32_t)H.pwr_wait_ms + 100U) H.state = S_SCAN;
            break;

        case S_SCAN:
            if (step(ctl(ph, USB_D2H | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_OTHER,
                         HUB_REQ_GET_STATUS, 0, H.port, s_ctl, 4),
                     S_SCAN))
            {
                H.port_status = le16(&s_ctl[0]);
                H.port_change = le16(&s_ctl[2]);
                if ((H.port_status & PS_CONNECTION) && !(H.skip & (1U << H.port)))
                {
                    if (H.port_status & PS_LOW_SPEED) { fail(USBH_NOT_SUPPORTED); break; }
                    H.state = S_CLR_C_CONN;
                }
                else if (++H.port > H.ports)
                {
                    H.port  = 1;
                    H.t0    = now;
                    H.state = S_SCAN_WAIT;
                }
            }
            break;

        case S_SCAN_WAIT:
            if (now - H.t0 >= 100U) H.state = S_SCAN;
            break;

        case S_CLR_C_CONN:
            step(ctl(ph, USB_H2D | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_OTHER,
                     HUB_REQ_CLEAR_FEATURE, C_PORT_CONNECTION, H.port, 0, 0),
                 S_RESET);
            break;

        case S_RESET:
            if (step(ctl(ph, USB_H2D | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_OTHER,
                         HUB_REQ_SET_FEATURE, PORT_RESET, H.port, 0, 0),
                     S_RESET_POLL))
                H.t0 = now;
            break;

        case S_RESET_POLL:
            if (now - H.t0 < 20U) break; /* the reset lasts 10-20 ms */
            if (step(ctl(ph, USB_D2H | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_OTHER,
                         HUB_REQ_GET_STATUS, 0, H.port, s_ctl, 4),
                     S_RESET_POLL))
            {
                H.port_status = le16(&s_ctl[0]);
                H.port_change = le16(&s_ctl[2]);
                if (!(H.port_status & PS_RESET) && (H.port_status & PS_ENABLE))
                    H.state = S_CLR_C_RESET;
                else if (now - H.t0 > 500U)
                    fail(USBH_FAIL);
                else
                    H.t0 = now - 10U; /* poll again in 10 ms */
            }
            break;

        case S_CLR_C_RESET:
            if (step(ctl(ph, USB_H2D | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_OTHER,
                         HUB_REQ_CLEAR_FEATURE, C_PORT_RESET, H.port, 0, 0),
                     S_RECOVERY))
                H.t0 = now;
            break;

        case S_RECOVERY:
            /* Reset recovery, then talk to the new device at address 0. */
            if (now - H.t0 >= 20U)
            {
                ctl_target(ph, 0, 8);
                H.state = S_DEV_DESC8;
            }
            break;

        case S_DEV_DESC8:
            if (step(USBH_GetDescriptor(ph, USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD,
                                        USB_DESC_DEVICE, s_ctl, 8),
                     S_DEV_ADDR))
            {
                H.dev_mps0 = s_ctl[7] ? s_ctl[7] : 8;
                ctl_target(ph, 0, H.dev_mps0);
            }
            break;

        case S_DEV_ADDR:
            if (step(USBH_SetAddress(ph, H.dev_addr), S_DEV_ADDR_WAIT)) H.t0 = now;
            break;

        case S_DEV_ADDR_WAIT:
            if (now - H.t0 >= 5U)
            {
                ctl_target(ph, H.dev_addr, H.dev_mps0);
                H.state = S_DEV_DESC18;
            }
            break;

        case S_DEV_DESC18:
            if (step(USBH_GetDescriptor(ph, USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD,
                                        USB_DESC_DEVICE, s_ctl, 18),
                     S_DEV_CFG9))
            {
                H.vid = le16(&s_ctl[8]);
                H.pid = le16(&s_ctl[10]);
            }
            break;

        case S_DEV_CFG9:
            if (step(USBH_GetDescriptor(ph, USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD,
                                        USB_DESC_CONFIGURATION, s_ctl, 9),
                     S_DEV_CFG_FULL))
            {
                H.cfg_total = le16(&s_ctl[2]);
                H.cfg_value = s_ctl[5];
                if (H.cfg_total > sizeof s_ctl) H.cfg_total = sizeof s_ctl;
            }
            break;

        case S_DEV_CFG_FULL:
            if (step(USBH_GetDescriptor(ph, USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD,
                                        USB_DESC_CONFIGURATION, s_ctl, H.cfg_total),
                     S_DEV_SET_CFG))
            {
                if (!parse_cfg(s_ctl, H.cfg_total)) fail(USBH_NOT_SUPPORTED);
            }
            break;

        case S_DEV_SET_CFG:
            if (step(USBH_SetCfg(ph, H.cfg_value), S_RX))
            {
                H.in_pipe = USBH_AllocPipe(ph, H.in_ep);
                USBH_OpenPipe(ph, H.in_pipe, H.in_ep, H.dev_addr, ph->device.speed,
                              USBH_EP_BULK, H.in_size);
                USBH_LL_SetToggle(ph, H.in_pipe, 0U);
                H.out_pipe = USBH_AllocPipe(ph, H.out_ep);
                USBH_OpenPipe(ph, H.out_pipe, H.out_ep, H.dev_addr, ph->device.speed,
                              USBH_EP_BULK, H.out_size);
                USBH_LL_SetToggle(ph, H.out_pipe, 0U);
                H.ready = 1;
            }
            break;

        case S_RX:
            USBH_BulkReceiveData(ph, s_rx, H.in_size, H.in_pipe);
            H.state = S_RX_POLL;
            break;

        case S_RX_POLL:
        {
            const USBH_URBStateTypeDef u = USBH_LL_GetURBState(ph, H.in_pipe);
            if (u == USBH_URB_DONE)
            {
                const uint32_t n = USBH_LL_GetLastXferSize(ph, H.in_pipe);
                H.state = S_RX;
                if (s_cb) s_cb(s_rx, n, s_user);
            }
            else if (u == USBH_URB_ERROR || u == USBH_URB_STALL)
            {
                H.state = S_RX; /* re-arm; an unplug will surface as a disconnect */
            }
            break;
        }

        case S_REJECT:
            /* Back to the hub, turn the port off, never pick it again. A
             * fresh address for the next device, so nothing stale answers. */
            if (ph->RequestState == CMD_SEND) /* before each new request */
                ctl_target(ph, USBH_DEVICE_ADDRESS, H.hub_mps);
            if (step(ctl(ph, USB_H2D | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_OTHER,
                         HUB_REQ_CLEAR_FEATURE, PORT_ENABLE, H.port, 0, 0),
                     S_SCAN))
            {
                H.skip |= (uint16_t)(1U << H.port);
                H.rejects++;
                H.dev_addr = (uint8_t)(H.dev_addr + 1U);
                H.port     = 1;
            }
            else if (H.state == S_FAILED)
            {
                /* The hub itself would not answer: give up on this port
                 * anyway rather than the whole hub. */
                H.skip |= (uint16_t)(1U << H.port);
                H.rejects++;
                H.dev_addr = (uint8_t)(H.dev_addr + 1U);
                H.port     = 1;
                H.state    = S_SCAN;
            }
            break;

        default: break;
    }
    return USBH_OK;
}

static USBH_StatusTypeDef SOFProcess(USBH_HandleTypeDef *ph)
{
    (void)ph;
    return USBH_OK;
}

USBH_ClassTypeDef USBH_hub_midi = {
    "HUB-MIDI", HUB_CLASS, Init, DeInit, Requests, Process, SOFProcess, NULL,
};

/* ---------------------------------------------------------------- API */

uint8_t HUBMIDI_Ready(USBH_HandleTypeDef *ph)
{
    return ph->pActiveClass == &USBH_hub_midi && H.ready;
}

uint16_t HUBMIDI_OutSize(USBH_HandleTypeDef *ph)
{
    (void)ph;
    return H.ready ? H.out_size : 0;
}

USBH_StatusTypeDef HUBMIDI_Transmit(USBH_HandleTypeDef *ph, uint8_t *data, uint16_t len)
{
    if (!HUBMIDI_Ready(ph)) return USBH_FAIL;
    const USBH_URBStateTypeDef u = USBH_LL_GetURBState(ph, H.out_pipe);
    if (u != USBH_URB_IDLE && u != USBH_URB_DONE)
        return USBH_BUSY; /* in flight, or halted: never send into a halted pipe */
    if (len > H.out_size) len = H.out_size;
    USBH_BulkSendData(ph, data, len, H.out_pipe, 1U);
    return USBH_OK;
}

void HUBMIDI_SetReceiveCallback(HUBMIDI_RxCallback cb, void *user)
{
    s_cb   = cb;
    s_user = user;
}

HUBMIDI_Info HUBMIDI_GetInfo(void)
{
    HUBMIDI_Info i;
    i.state       = H.state;
    i.ports       = H.ports;
    i.port        = H.port;
    i.port_status = H.port_status;
    i.vid         = H.vid;
    i.pid         = H.pid;
    i.fail_state  = H.fail_state;
    i.fail_code   = H.fail_code;
    i.skipped     = H.skip;
    return i;
}

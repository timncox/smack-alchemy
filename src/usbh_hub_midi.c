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

/* Enumeration / port-management states (numbers appear in /lpdiag.txt). */
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
    S_UNUSED18,
    S_UNUSED19,
    S_FAILED,
    S_REJECT, /* the device is not MIDI: switch its port off, scan on */
    S_FULL,   /* every device slot taken: nothing more to scan for */
};

/* Per-device receive loop. */
enum { RX_ARM = 0, RX_POLL = 1 };

typedef struct
{
    uint8_t  ready;
    uint8_t  port, addr;
    uint8_t  in_ep, out_ep, in_pipe, out_pipe;
    uint16_t in_size, out_size;
    uint8_t  rx;
    uint16_t vid, pid;
    uint8_t  kind;     /* HUBMIDI_KIND_* */
    uint8_t  interval; /* interrupt IN poll period, ms (XInput) */
    uint32_t armed_ms; /* when the interrupt IN was last armed */
    uint32_t arms, dones, naks, errs;
} Dev;

typedef struct
{
    uint8_t  state;
    uint8_t  ports;
    uint16_t pwr_wait_ms;
    uint8_t  port;      /* port being scanned / enumerated, 1-based */
    uint32_t t0;
    uint16_t port_status, port_change;
    uint16_t skip;      /* ports whose device was not MIDI (bit n = port n) */
    uint16_t done;      /* ports carrying a configured device */
    uint8_t  next_addr; /* address for the next device enumerated */
    uint8_t  hub_mps;   /* the hub's control max packet size */
    uint8_t  ctl_addr;  /* where the control pipes point now */
    uint8_t  rejects;
    /* the device being enumerated */
    uint8_t  e_addr, e_mps0, e_cfg_value;
    uint16_t e_cfg_total, e_vid, e_pid;
    uint8_t  e_in_ep, e_out_ep, e_kind, e_interval;
    uint16_t e_in_size, e_out_size;
    uint8_t  fail_state, fail_code;
    Dev      dev[HUBMIDI_MAX_DEVICES];
} HubMidi;

static HubMidi            H;
static HUBMIDI_RxCallback s_cb;
static void              *s_user;

/* Control-transfer data and the MIDI receive buffers: DMA-reachable. */
static uint8_t DMA_SECTION s_ctl[512];
static uint8_t DMA_SECTION s_rx[HUBMIDI_MAX_DEVICES][64];

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

/* Point the core's control pipes at an address -- only between requests. */
static void ctl_to(USBH_HandleTypeDef *ph, uint8_t addr, uint8_t mps)
{
    if (ph->RequestState != CMD_SEND) return;
    if (H.ctl_addr == addr && ph->Control.pipe_size == mps) return;
    H.ctl_addr            = addr;
    ph->Control.pipe_size = mps;
    USBH_OpenPipe(ph, ph->Control.pipe_in, 0x80U, addr, ph->device.speed, USBH_EP_CONTROL, mps);
    USBH_OpenPipe(ph, ph->Control.pipe_out, 0x00U, addr, ph->device.speed, USBH_EP_CONTROL, mps);
}

static void to_hub(USBH_HandleTypeDef *ph) { ctl_to(ph, USBH_DEVICE_ADDRESS, H.hub_mps); }

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

static int free_slot(void)
{
    for (int i = 0; i < HUBMIDI_MAX_DEVICES; i++)
        if (!H.dev[i].ready) return i;
    return -1;
}

/* Find the first MIDI-streaming interface's bulk endpoints, or an XInput
 * interface's interrupt IN endpoint. */
static int parse_cfg(const uint8_t *c, uint16_t total)
{
    uint16_t i    = 0;
    int      midi = 0, pad = 0;
    H.e_in_ep = H.e_out_ep = 0;
    H.e_kind = HUBMIDI_KIND_MIDI;
    H.e_interval = 1;
    while (i + 2 <= total)
    {
        const uint8_t len = c[i], type = c[i + 1];
        if (len < 2 || i + len > total) break;
        if (type == 0x04 && len >= 9) /* interface */
        {
            if (midi && H.e_in_ep && H.e_out_ep) break;
            if (pad && H.e_in_ep) break;
            midi = (c[i + 5] == 0x01 && c[i + 6] == 0x03);
            pad  = (c[i + 5] == 0xFF && c[i + 6] == 0x5D && c[i + 7] == 0x01);
        }
        else if (type == 0x05 && len >= 7 && pad && (c[i + 3] & 0x03) == 0x03
                 && (c[i + 2] & 0x80U) && !H.e_in_ep)
        {
            H.e_in_ep    = c[i + 2];
            H.e_in_size  = le16(&c[i + 4]) & 0x03FFU;
            H.e_interval = c[i + 6] ? c[i + 6] : 1;
            H.e_kind     = HUBMIDI_KIND_XINPUT;
        }
        else if (type == 0x05 && len >= 7 && midi && (c[i + 3] & 0x03) == 0x02)
        {
            const uint8_t  ep  = c[i + 2];
            const uint16_t mps = le16(&c[i + 4]) & 0x03FFU;
            if (ep & 0x80U) { if (!H.e_in_ep)  { H.e_in_ep = ep;  H.e_in_size = mps; } }
            else            { if (!H.e_out_ep) { H.e_out_ep = ep; H.e_out_size = mps; } }
        }
        i = (uint16_t)(i + len);
    }
    if (H.e_in_size > sizeof s_rx[0]) H.e_in_size = sizeof s_rx[0];
    if (H.e_kind == HUBMIDI_KIND_XINPUT) return H.e_in_ep != 0;
    return H.e_in_ep && H.e_out_ep;
}

/* On to the next port; after the last, wait and go round again. */
static void next_port(uint32_t now)
{
    if (++H.port > H.ports)
    {
        H.port  = 1;
        H.t0    = now;
        H.state = S_SCAN_WAIT;
    }
    else
        H.state = S_SCAN;
}

/* ---------------------------------------------------------------- per-device RX */

static void rx_step(USBH_HandleTypeDef *ph, int i)
{
    Dev *d = &H.dev[i];
    if (!d->ready) return;
    if (d->rx == RX_ARM)
    {
        if (d->kind == HUBMIDI_KIND_XINPUT)
        {
            /* A NAK halts an interrupt channel (URB_NOTREADY) and nothing
             * re-arms it: poll again once per bInterval, like the HID class. */
            const uint32_t now = HAL_GetTick();
            if (now - d->armed_ms < d->interval) return;
            d->armed_ms = now;
            USBH_InterruptReceiveData(ph, s_rx[i], (uint8_t)d->in_size, d->in_pipe);
        }
        else
            USBH_BulkReceiveData(ph, s_rx[i], d->in_size, d->in_pipe);
        d->arms++;
        d->rx = RX_POLL;
        return;
    }
    const USBH_URBStateTypeDef u = USBH_LL_GetURBState(ph, d->in_pipe);
    if (u == USBH_URB_DONE)
    {
        const uint32_t n = USBH_LL_GetLastXferSize(ph, d->in_pipe);
        d->dones++;
        d->rx = RX_ARM;
        if (s_cb) s_cb((uint8_t)i, s_rx[i], n, s_user);
    }
    else if (u == USBH_URB_ERROR || u == USBH_URB_STALL
             || (u == USBH_URB_NOTREADY && d->kind == HUBMIDI_KIND_XINPUT))
    {
        if (u == USBH_URB_NOTREADY) d->naks++;
        else d->errs++;
        d->rx = RX_ARM; /* re-arm; an unplug surfaces as a disconnect */
    }
}

/* ---------------------------------------------------------------- class */

static USBH_StatusTypeDef Init(USBH_HandleTypeDef *ph)
{
    memset(&H, 0, sizeof H);
    H.state                 = S_HUB_DESC;
    H.hub_mps               = (uint8_t)ph->Control.pipe_size;
    H.ctl_addr              = USBH_DEVICE_ADDRESS;
    H.next_addr             = DEV_ADDR0;
    ph->pActiveClass->pData = &H;
    return USBH_OK;
}

static USBH_StatusTypeDef DeInit(USBH_HandleTypeDef *ph)
{
    for (int i = 0; i < HUBMIDI_MAX_DEVICES; i++)
    {
        Dev *d = &H.dev[i];
        if (d->in_pipe)  { USBH_ClosePipe(ph, d->in_pipe);  USBH_FreePipe(ph, d->in_pipe);  }
        if (d->out_pipe) { USBH_ClosePipe(ph, d->out_pipe); USBH_FreePipe(ph, d->out_pipe); }
        memset(d, 0, sizeof *d);
    }
    if (ph->pActiveClass) ph->pActiveClass->pData = 0;
    return USBH_OK;
}

static USBH_StatusTypeDef Requests(USBH_HandleTypeDef *ph)
{
    (void)ph;
    return USBH_OK; /* all the work is in the background process */
}

static void enum_step(USBH_HandleTypeDef *ph, uint32_t now)
{
    switch (H.state)
    {
        case S_HUB_DESC:
            to_hub(ph);
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
            to_hub(ph);
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
            if (free_slot() < 0) { H.state = S_FULL; break; }
            if ((H.skip | H.done) & (1U << H.port)) { next_port(now); break; }
            to_hub(ph);
            if (step(ctl(ph, USB_D2H | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_OTHER,
                         HUB_REQ_GET_STATUS, 0, H.port, s_ctl, 4),
                     S_SCAN))
            {
                H.port_status = le16(&s_ctl[0]);
                H.port_change = le16(&s_ctl[2]);
                if (!(H.port_status & PS_CONNECTION))
                    next_port(now);
                else if (H.port_status & PS_LOW_SPEED)
                {
                    H.skip |= (uint16_t)(1U << H.port); /* would need PRE packets */
                    next_port(now);
                }
                else
                    H.state = S_CLR_C_CONN;
            }
            break;

        case S_SCAN_WAIT:
            /* Free ports are looked at again twice a second, so a controller
             * plugged in later is picked up. */
            if (now - H.t0 >= 500U) H.state = S_SCAN;
            break;

        case S_CLR_C_CONN:
            to_hub(ph);
            step(ctl(ph, USB_H2D | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_OTHER,
                     HUB_REQ_CLEAR_FEATURE, C_PORT_CONNECTION, H.port, 0, 0),
                 S_RESET);
            break;

        case S_RESET:
            to_hub(ph);
            if (step(ctl(ph, USB_H2D | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_OTHER,
                         HUB_REQ_SET_FEATURE, PORT_RESET, H.port, 0, 0),
                     S_RESET_POLL))
                H.t0 = now;
            break;

        case S_RESET_POLL:
            if (now - H.t0 < 20U) break; /* the reset lasts 10-20 ms */
            to_hub(ph);
            if (step(ctl(ph, USB_D2H | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_OTHER,
                         HUB_REQ_GET_STATUS, 0, H.port, s_ctl, 4),
                     S_RESET_POLL))
            {
                H.port_status = le16(&s_ctl[0]);
                H.port_change = le16(&s_ctl[2]);
                if (!(H.port_status & PS_RESET) && (H.port_status & PS_ENABLE))
                    H.state = S_CLR_C_RESET;
                else if (now - H.t0 > 500U)
                {
                    H.state = S_DEV_DESC8; /* so fail() rejects the port, not the hub */
                    fail(USBH_FAIL);
                }
                else
                    H.t0 = now - 10U; /* poll again in 10 ms */
            }
            break;

        case S_CLR_C_RESET:
            to_hub(ph);
            if (step(ctl(ph, USB_H2D | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_OTHER,
                         HUB_REQ_CLEAR_FEATURE, C_PORT_RESET, H.port, 0, 0),
                     S_RECOVERY))
                H.t0 = now;
            break;

        case S_RECOVERY:
            /* Reset recovery; the new device answers at address 0. */
            if (now - H.t0 >= 20U)
            {
                H.e_addr = H.next_addr;
                H.state  = S_DEV_DESC8;
            }
            break;

        case S_DEV_DESC8:
            ctl_to(ph, 0, 8);
            if (step(USBH_GetDescriptor(ph, USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD,
                                        USB_DESC_DEVICE, s_ctl, 8),
                     S_DEV_ADDR))
                H.e_mps0 = s_ctl[7] ? s_ctl[7] : 8;
            break;

        case S_DEV_ADDR:
            ctl_to(ph, 0, H.e_mps0);
            if (step(USBH_SetAddress(ph, H.e_addr), S_DEV_ADDR_WAIT)) H.t0 = now;
            break;

        case S_DEV_ADDR_WAIT:
            if (now - H.t0 >= 5U) H.state = S_DEV_DESC18;
            break;

        case S_DEV_DESC18:
            ctl_to(ph, H.e_addr, H.e_mps0);
            if (step(USBH_GetDescriptor(ph, USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD,
                                        USB_DESC_DEVICE, s_ctl, 18),
                     S_DEV_CFG9))
            {
                H.e_vid = le16(&s_ctl[8]);
                H.e_pid = le16(&s_ctl[10]);
            }
            break;

        case S_DEV_CFG9:
            ctl_to(ph, H.e_addr, H.e_mps0);
            if (step(USBH_GetDescriptor(ph, USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD,
                                        USB_DESC_CONFIGURATION, s_ctl, 9),
                     S_DEV_CFG_FULL))
            {
                H.e_cfg_total = le16(&s_ctl[2]);
                H.e_cfg_value = s_ctl[5];
                if (H.e_cfg_total > sizeof s_ctl) H.e_cfg_total = sizeof s_ctl;
            }
            break;

        case S_DEV_CFG_FULL:
            ctl_to(ph, H.e_addr, H.e_mps0);
            if (step(USBH_GetDescriptor(ph, USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD,
                                        USB_DESC_CONFIGURATION, s_ctl, H.e_cfg_total),
                     S_DEV_SET_CFG))
            {
                if (!parse_cfg(s_ctl, H.e_cfg_total))
                {
                    H.state = S_DEV_CFG_FULL; /* so fail() rejects the port */
                    fail(USBH_NOT_SUPPORTED);
                }
            }
            break;

        case S_DEV_SET_CFG:
        {
            const int slot = free_slot();
            if (slot < 0) { H.state = S_FULL; break; }
            ctl_to(ph, H.e_addr, H.e_mps0);
            if (step(USBH_SetCfg(ph, H.e_cfg_value), S_SCAN))
            {
                Dev *d      = &H.dev[slot];
                memset(d, 0, sizeof *d);
                d->port     = H.port;
                d->addr     = H.e_addr;
                d->vid      = H.e_vid;
                d->pid      = H.e_pid;
                d->in_ep    = H.e_in_ep;
                d->out_ep   = H.e_out_ep;
                d->in_size  = H.e_in_size;
                d->out_size = H.e_out_size;
                d->kind     = H.e_kind;
                d->interval = H.e_interval;
                d->in_pipe  = USBH_AllocPipe(ph, d->in_ep);
                USBH_OpenPipe(ph, d->in_pipe, d->in_ep, d->addr, ph->device.speed,
                              d->kind == HUBMIDI_KIND_XINPUT ? USBH_EP_INTERRUPT : USBH_EP_BULK,
                              d->in_size);
                USBH_LL_SetToggle(ph, d->in_pipe, 0U);
                if (d->kind == HUBMIDI_KIND_MIDI)
                {
                    d->out_pipe = USBH_AllocPipe(ph, d->out_ep);
                    USBH_OpenPipe(ph, d->out_pipe, d->out_ep, d->addr, ph->device.speed,
                                  USBH_EP_BULK, d->out_size);
                    USBH_LL_SetToggle(ph, d->out_pipe, 0U);
                }
                d->rx       = RX_ARM;
                d->ready    = 1;
                H.done     |= (uint16_t)(1U << H.port);
                H.next_addr = (uint8_t)(H.next_addr + 1U);
                next_port(now); /* scan on for a second controller */
            }
            break;
        }

        case S_REJECT:
        {
            /* Back to the hub, turn the port off, never pick it again. A
             * fresh address for the next device, so nothing stale answers. */
            to_hub(ph);
            const USBH_StatusTypeDef st =
                ctl(ph, USB_H2D | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_OTHER,
                    HUB_REQ_CLEAR_FEATURE, PORT_ENABLE, H.port, 0, 0);
            if (st == USBH_BUSY) break;
            /* done, or the hub would not answer: give up the port either way */
            H.skip     |= (uint16_t)(1U << H.port);
            H.rejects++;
            H.next_addr = (uint8_t)(H.next_addr + 1U);
            next_port(now);
            break;
        }

        default: break; /* S_FAILED, S_FULL */
    }
}

static USBH_StatusTypeDef Process(USBH_HandleTypeDef *ph)
{
    const uint32_t now = HAL_GetTick();
    enum_step(ph, now);
    for (int i = 0; i < HUBMIDI_MAX_DEVICES; i++) rx_step(ph, i);
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

static int active(USBH_HandleTypeDef *ph) { return ph->pActiveClass == &USBH_hub_midi; }

uint8_t HUBMIDI_Ready(USBH_HandleTypeDef *ph)
{
    if (!active(ph)) return 0;
    for (int i = 0; i < HUBMIDI_MAX_DEVICES; i++)
        if (H.dev[i].ready) return 1;
    return 0;
}

uint8_t HUBMIDI_DevReady(USBH_HandleTypeDef *ph, uint8_t dev)
{
    return active(ph) && dev < HUBMIDI_MAX_DEVICES && H.dev[dev].ready;
}

void HUBMIDI_DevId(uint8_t dev, uint16_t *vid, uint16_t *pid)
{
    if (dev >= HUBMIDI_MAX_DEVICES) { *vid = *pid = 0; return; }
    *vid = H.dev[dev].vid;
    *pid = H.dev[dev].pid;
}

uint8_t HUBMIDI_DevKind(uint8_t dev)
{
    return dev < HUBMIDI_MAX_DEVICES ? H.dev[dev].kind : HUBMIDI_KIND_MIDI;
}

uint16_t HUBMIDI_OutSize(USBH_HandleTypeDef *ph, uint8_t dev)
{
    return HUBMIDI_DevReady(ph, dev) && H.dev[dev].kind == HUBMIDI_KIND_MIDI
               ? H.dev[dev].out_size : 0;
}

USBH_StatusTypeDef HUBMIDI_Transmit(USBH_HandleTypeDef *ph, uint8_t dev, uint8_t *data,
                                    uint16_t len)
{
    if (!HUBMIDI_DevReady(ph, dev) || H.dev[dev].kind != HUBMIDI_KIND_MIDI) return USBH_FAIL;
    const Dev                 *d = &H.dev[dev];
    const USBH_URBStateTypeDef u = USBH_LL_GetURBState(ph, d->out_pipe);
    if (u != USBH_URB_IDLE && u != USBH_URB_DONE)
        return USBH_BUSY; /* in flight, or halted: never send into a halted pipe */
    if (len > d->out_size) len = d->out_size;
    USBH_BulkSendData(ph, data, len, d->out_pipe, 1U);
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
    memset(&i, 0, sizeof i);
    i.state       = H.state;
    i.ports       = H.ports;
    i.port        = H.port;
    i.port_status = H.port_status;
    i.fail_state  = H.fail_state;
    i.fail_code   = H.fail_code;
    i.skipped     = H.skip;
    i.done        = H.done;
    for (int d = 0; d < HUBMIDI_MAX_DEVICES; d++)
    {
        i.dev_vid[d]  = H.dev[d].vid;
        i.dev_pid[d]  = H.dev[d].pid;
        i.dev_port[d] = H.dev[d].ready ? H.dev[d].port : 0;
        i.dev_kind[d] = H.dev[d].kind;
        i.rx_arms[d]  = H.dev[d].arms;
        i.rx_done[d]  = H.dev[d].dones;
        i.rx_nak[d]   = H.dev[d].naks;
        i.rx_err[d]   = H.dev[d].errs;
    }
    return i;
}

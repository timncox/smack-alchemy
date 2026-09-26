/* usb_audio -- see usb_audio.h. */
#include "usb_audio.h"

#include <string.h>

#include "stm32h7xx_hal.h"
#include "usbd_core.h"
#include "usbd_ctlreq.h"
#include "usbd_ioreq.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_HS; /* libDaisy usbd_conf.c */

/* ---- format ---------------------------------------------------------------- */

#define UAC_RATE        48000u
#define UAC_CH          2u
#define UAC_SUBFRAME    3u                        /* 24-bit                 */
#define UAC_FRAME_BYTES (UAC_CH * UAC_SUBFRAME)   /* 6                      */
#define UAC_NOM_FRAMES  (UAC_RATE / 1000u)        /* 48 per 1 ms frame      */
#define UAC_MAX_FRAMES  (UAC_NOM_FRAMES + 1u)
#define UAC_PKT_MAX     (UAC_MAX_FRAMES * UAC_FRAME_BYTES) /* 294          */

#define EP_OUT      0x01u   /* audio from the computer                      */
#define EP_IN       0x82u   /* audio to the computer                        */

#define ITF_AC      0u
#define ITF_OUT     1u
#define ITF_IN      2u

/* Ring sizing (frames). The audio callback moves 128 frames at a time on
 * Mark (Smack too), so the fills swing by a block; the targets sit a block
 * plus ~2.7 ms of slack away from empty. */
#define RING        1024u
#define RING_MASK   (RING - 1u)
#define OUT_PRIME   256u
#define OUT_TARGET  192.0f
#define IN_PRIME    192u
#define IN_TARGET   192.0f

/* ---- descriptors ----------------------------------------------------------- */

#define LO(x) (uint8_t)((x) & 0xFFu)
#define HI(x) (uint8_t)(((x) >> 8) & 0xFFu)

#define AC_CS_LEN   (10u + 12u + 9u + 12u + 9u)                /* 52 */
#define AS_OUT_LEN  (9u + 9u + 7u + 11u + 9u + 7u)             /* 52 */
#define AS_IN_LEN   (9u + 9u + 7u + 11u + 9u + 7u)             /* 52 */
#define CFG_LEN     (9u + 9u + AC_CS_LEN + AS_OUT_LEN + AS_IN_LEN)

#define FORMAT_I                                                              \
    0x0B, 0x24, 0x02, 0x01, UAC_CH, UAC_SUBFRAME, 24, 0x01,                   \
    LO(UAC_RATE), HI(UAC_RATE), (uint8_t)(UAC_RATE >> 16)

static const uint8_t k_cfg[CFG_LEN] = {
    /* configuration */
    0x09, USB_DESC_TYPE_CONFIGURATION, LO(CFG_LEN), HI(CFG_LEN),
    0x03,               /* interfaces                                 */
    0x01, 0x00,         /* value, string                              */
    0xC0,               /* self-powered (the rack)                    */
    0x32,               /* 100 mA                                     */

    /* interface 0: AudioControl */
    0x09, USB_DESC_TYPE_INTERFACE, ITF_AC, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,
    /* AC header, 2 streaming interfaces */
    0x0A, 0x24, 0x01, 0x00, 0x01, LO(AC_CS_LEN), HI(AC_CS_LEN), 0x02,
    ITF_OUT, ITF_IN,
    /* IT 1: USB streaming, stereo */
    0x0C, 0x24, 0x02, 0x01, 0x01, 0x01, 0x00, UAC_CH, 0x03, 0x00, 0x00, 0x00,
    /* OT 3: line connector (J9/J10), from IT 1 */
    0x09, 0x24, 0x03, 0x03, 0x03, 0x06, 0x00, 0x01, 0x00,
    /* IT 4: line connector (J1/J2), stereo */
    0x0C, 0x24, 0x02, 0x04, 0x03, 0x06, 0x00, UAC_CH, 0x03, 0x00, 0x00, 0x00,
    /* OT 6: USB streaming, from IT 4 */
    0x09, 0x24, 0x03, 0x06, 0x01, 0x01, 0x00, 0x04, 0x00,

    /* interface 1 alt 0: OUT, zero bandwidth */
    0x09, USB_DESC_TYPE_INTERFACE, ITF_OUT, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00,
    /* interface 1 alt 1: OUT */
    0x09, USB_DESC_TYPE_INTERFACE, ITF_OUT, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00,
    /* AS general: terminal 1, PCM */
    0x07, 0x24, 0x01, 0x01, 0x01, 0x01, 0x00,
    FORMAT_I,
    /* EP 0x01: isochronous, ADAPTIVE: the codec's clock is steered to the
     * host's (PLL3, below), so there is no feedback endpoint. The first
     * build had one; macOS polls it every 2 ms, and a few missed polls a
     * second made its driver fail to queue the next feedback transfer
     * ("enqueueAvailableFeedbackTransferBlocks_error 0xe00002ee") and
     * restart the whole stream -- a burst of clicks every ~30 s. */
    0x09, USB_DESC_TYPE_ENDPOINT, EP_OUT, 0x09, LO(UAC_PKT_MAX), HI(UAC_PKT_MAX),
    0x01, 0x00, 0x00,
    /* CS endpoint: no controls */
    0x07, 0x25, 0x01, 0x00, 0x00, 0x00, 0x00,

    /* interface 2 alt 0: IN, zero bandwidth */
    0x09, USB_DESC_TYPE_INTERFACE, ITF_IN, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00,
    /* interface 2 alt 1: IN */
    0x09, USB_DESC_TYPE_INTERFACE, ITF_IN, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00,
    /* AS general: terminal 6, PCM */
    0x07, 0x24, 0x01, 0x06, 0x01, 0x01, 0x00,
    FORMAT_I,
    /* EP 0x82: isochronous, synchronous (48 frames every frame) */
    0x09, USB_DESC_TYPE_ENDPOINT, EP_IN, 0x0D, LO(UAC_PKT_MAX), HI(UAC_PKT_MAX),
    0x01, 0x00, 0x00,
    0x07, 0x25, 0x01, 0x00, 0x00, 0x00, 0x00,
};
_Static_assert(sizeof k_cfg == CFG_LEN, "configuration descriptor length");
_Static_assert(CFG_LEN == 174u, "UAC1 2-in/2-out, no feedback, is 174 bytes");

static uint8_t k_dev[USB_LEN_DEV_DESC] = {
    0x12, USB_DESC_TYPE_DEVICE,
    0x10, 0x01,             /* USB 1.1: full speed only, no qualifier     */
    0x00, 0x00, 0x00,       /* class per interface                        */
    USB_MAX_EP0_SIZE,
    0x83, 0x04,             /* 0x0483 */
    0x30, 0x57,             /* 0x5730: not HostLink's 0x5740, so the Mac
                             * does not reuse the CDC device's entry      */
    0x01, 0x01,             /* bcdDevice 1.01 (1.00 had a feedback EP)    */
    USBD_IDX_MFC_STR, USBD_IDX_PRODUCT_STR, USBD_IDX_SERIAL_STR,
    0x01,
};

static uint8_t k_langid[USB_LEN_LANGID_STR_DESC] = {
    USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING, 0x09, 0x04};

static const char *s_product = "Alchemy Lab";
static uint8_t     s_str[2u + 2u * 48u];

static uint8_t *d_device(USBD_SpeedTypeDef s, uint16_t *len)
{
    (void)s;
    *len = sizeof k_dev;
    return k_dev;
}
static uint8_t *d_langid(USBD_SpeedTypeDef s, uint16_t *len)
{
    (void)s;
    *len = sizeof k_langid;
    return k_langid;
}
static uint8_t *d_string(const char *text, uint16_t *len)
{
    USBD_GetString((uint8_t *)(uintptr_t)text, s_str, len);
    return s_str;
}
static uint8_t *d_mfc(USBD_SpeedTypeDef s, uint16_t *len)
{
    (void)s;
    return d_string("Alchemy Lab", len);
}
static uint8_t *d_product(USBD_SpeedTypeDef s, uint16_t *len)
{
    (void)s;
    return d_string(s_product, len);
}
static uint8_t *d_serial(USBD_SpeedTypeDef s, uint16_t *len)
{
    (void)s;
    static char hex[25];
    const uint32_t w[3] = {HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2()};
    for (int i = 0; i < 24; i++)
    {
        const uint32_t nib = (w[i / 8] >> (28 - 4 * (i % 8))) & 0xFu;
        hex[i] = (char)(nib < 10 ? '0' + nib : 'A' + nib - 10);
    }
    hex[24] = 0;
    return d_string(hex, len);
}
static uint8_t *d_config_str(USBD_SpeedTypeDef s, uint16_t *len)
{
    (void)s;
    return d_string("Audio", len);
}

static USBD_DescriptorsTypeDef s_desc = {
    .GetDeviceDescriptor           = d_device,
    .GetLangIDStrDescriptor        = d_langid,
    .GetManufacturerStrDescriptor  = d_mfc,
    .GetProductStrDescriptor       = d_product,
    .GetSerialStrDescriptor        = d_serial,
    .GetConfigurationStrDescriptor = d_config_str,
    .GetInterfaceStrDescriptor     = d_config_str,
};

/* ---- state ------------------------------------------------------------------ */

static USBD_HandleTypeDef s_dev;
static volatile uint8_t   s_started;
static volatile uint8_t   s_reboot_req;

/* Frames as signed 24-bit in int32, L then R. */
static int32_t           s_out_ring[RING][UAC_CH];
static volatile uint32_t s_out_wr, s_out_rd;   /* USB writes, audio reads */
static int32_t           s_in_ring[RING][UAC_CH];
static volatile uint32_t s_in_wr, s_in_rd;     /* audio writes, USB reads */

static volatile uint8_t s_out_on, s_in_on;     /* alt settings            */
static volatile uint8_t s_out_primed, s_in_primed;
static volatile uint8_t s_in_busy;

static uint8_t s_out_pkt[UAC_PKT_MAX + 6u];
static uint8_t s_in_pkt[UAC_PKT_MAX];
static uint16_t s_in_len;                      /* bytes in s_in_pkt       */
static uint8_t s_ctl[64];

/* The codec's rate in frames per USB frame, measured against the CPU cycle
 * counter: the audio callback times >= 64k frames (~1.4 s) of codec, the
 * SOF handler times 1024 USB frames, and the ratio is the rate. Both spans
 * are long against the interrupt jitter (~1 us), so the estimate is good to
 * a few ppm, and the ring-fill terms only have to trim. Fill alone (the
 * first build) hunted: macOS smooths the feedback, and the lag turned the
 * loop into a ~26 s oscillation that underran the OUT ring. */
static volatile uint32_t s_codec_frames;
static uint32_t          s_cf_mark_frames, s_cf_mark_cyc;
static volatile float    s_cyc_per_frame;   /* codec, 0 = not measured yet */
static uint32_t          s_sof_mark_cyc;
static volatile float    s_cyc_per_sof;     /* USB, 0 = not measured yet   */
static float             s_rate = (float)UAC_NOM_FRAMES;
static float             s_in_frac;
static float             s_out_avg, s_in_avg;
/* The rate told to the host, 10.14: exactly 48.000, in the feedback and in
 * the IN packet sizes. macOS (usbaudiod, 2026-09) models the device at the
 * nominal rate whatever the feedback says and logs the drift
 * ("AUAAudioDevice_updateTimeStamp ioDriftNS"); libDaisy's audio PLL runs
 * the codec 250 ppm fast (49.167 MHz for 49.152), and every ~8 ms of drift
 * (~32 s) macOS skipped OUT packets for a few seconds. So the codec is made
 * to run at 48 kHz by the host's clock instead: see the PLL3 lock below. */
static uint32_t          s_fbq = UAC_NOM_FRAMES << 14;

/* ---- codec clock locked to USB ------------------------------------------------
 * PLL3 (HSE 16 MHz / M 6 = 2.667 MHz reference) feeds the SAI (and the ADC
 * and I2C4). libDaisy: N 295, no fraction = 786.667 MHz VCO. Here N 294 plus
 * a fraction (0..8191 of one step, one LSB = 0.414 ppm) is steered once a
 * second so the codec makes exactly 48.000 frames per USB frame, trimmed by
 * the ring fills (at most +-20 ppm) so the rings sit at their targets. */
#define PLL3_N_LOCKED   294u
#define PLL3_FRAC_START 7580u         /* 7471 = 49.152 MHz exactly; +116 for the
                                       * crystal gap measured on module 2 (~48 ppm) */
#define PLL3_PPM_PER_LSB 0.4139f
static volatile uint32_t s_pll_frac = PLL3_FRAC_START;
static float             s_pll_err_ppm;

static void pll3_set_frac(uint32_t frac)
{
    __HAL_RCC_PLL3FRACN_DISABLE();
    __HAL_RCC_PLL3FRACN_CONFIG(frac);
    __HAL_RCC_PLL3FRACN_ENABLE();      /* taken on the 0 -> 1 edge */
}

/* Once, before the SAI starts (no MCLK to the codec yet): N 295 -> 294 needs
 * the PLL off. The ADC and I2C4 lose their clock for the ~100 us relock. */
static void pll3_relock(void)
{
    __HAL_RCC_PLL3_DISABLE();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL3RDY)) {}
    const uint32_t divr = RCC->PLL3DIVR;
    RCC->PLL3DIVR = (divr & ~RCC_PLL3DIVR_N3) | ((PLL3_N_LOCKED - 1u) & RCC_PLL3DIVR_N3);
    __HAL_RCC_PLL3FRACN_DISABLE();
    __HAL_RCC_PLL3FRACN_CONFIG(PLL3_FRAC_START);
    __HAL_RCC_PLL3FRACN_ENABLE();
    __HAL_RCC_PLL3_ENABLE();
    while (!__HAL_RCC_GET_FLAG(RCC_FLAG_PLL3RDY)) {}
}

/* Once a second, from SOF: the measured codec rate against 48.000 (and the
 * ring trim), half of the error per step. */
static void pll3_steer(void)
{
    float err = 0.0f;   /* frames: positive = the codec is too slow */
    if (s_out_on && s_out_primed) err += s_out_avg - OUT_TARGET;
    if (s_in_on && s_in_primed)   err -= s_in_avg - IN_TARGET;
    float want_ppm = err * 0.05f;
    if (want_ppm > 20.0f)  want_ppm = 20.0f;
    if (want_ppm < -20.0f) want_ppm = -20.0f;
    const float have_ppm = (s_rate / (float)UAC_NOM_FRAMES - 1.0f) * 1e6f;
    s_pll_err_ppm = have_ppm - want_ppm;
    float step = -0.5f * s_pll_err_ppm / PLL3_PPM_PER_LSB;
    if (step > 400.0f)  step = 400.0f;
    if (step < -400.0f) step = -400.0f;
    int32_t f = (int32_t)s_pll_frac + (int32_t)(step >= 0 ? step + 0.5f : step - 0.5f);
    if (f < 0) f = 0;
    if (f > 8191) f = 8191;
    if ((uint32_t)f != s_pll_frac)
    {
        s_pll_frac = (uint32_t)f;
        pll3_set_frac(s_pll_frac);
    }
}

static volatile UAC_Info s_info;
UAC_Info UAC_GetInfo(void);

/* ---- helpers ---------------------------------------------------------------- */

static inline int32_t f2s24(float v)
{
    if (v > 0.99999988f) v = 0.99999988f;
    if (v < -1.0f)       v = -1.0f;
    return (int32_t)(v * 8388608.0f);
}

static inline void put24(uint8_t *p, int32_t s)
{
    p[0] = (uint8_t)s;
    p[1] = (uint8_t)(s >> 8);
    p[2] = (uint8_t)(s >> 16);
}

static inline int32_t get24(const uint8_t *p)
{
    int32_t s = (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16));
    return (s << 8) >> 8;
}

static void out_stream(USBD_HandleTypeDef *pdev, uint8_t on)
{
    if (on && !s_out_on)
    {
        USBD_LL_OpenEP(pdev, EP_OUT, USBD_EP_TYPE_ISOC, UAC_PKT_MAX);
        pdev->ep_out[EP_OUT & 0xFu].is_used = 1u;
        s_out_wr     = s_out_rd;          /* producer side: empty the ring */
        s_out_primed = 0u;
        s_out_avg    = OUT_TARGET;
        s_out_on     = 1u;
        USBD_LL_PrepareReceive(pdev, EP_OUT, s_out_pkt, UAC_PKT_MAX);
    }
    else if (!on && s_out_on)
    {
        s_out_on = 0u;
        USBD_LL_CloseEP(pdev, EP_OUT);
        pdev->ep_out[EP_OUT & 0xFu].is_used = 0u;
    }
}

static void in_stream(USBD_HandleTypeDef *pdev, uint8_t on)
{
    if (on && !s_in_on)
    {
        USBD_LL_OpenEP(pdev, EP_IN, USBD_EP_TYPE_ISOC, UAC_PKT_MAX);
        pdev->ep_in[EP_IN & 0xFu].is_used = 1u;
        USBD_LL_FlushEP(pdev, EP_IN);
        s_in_rd     = s_in_wr;            /* consumer side: empty the ring */
        s_in_primed = 0u;
        s_in_busy   = 0u;
        s_in_len    = 0u;
        s_in_frac   = 0.0f;
        s_in_avg    = IN_TARGET;
        s_in_on     = 1u;
    }
    else if (!on && s_in_on)
    {
        s_in_on = 0u;
        USBD_LL_FlushEP(pdev, EP_IN);
        USBD_LL_CloseEP(pdev, EP_IN);
        pdev->ep_in[EP_IN & 0xFu].is_used = 0u;
    }
}

/* ---- OUT endpoint arming --------------------------------------------------- */

static inline USB_OTG_OUTEndpointTypeDef *out_ep_regs(void)
{
    return (USB_OTG_OUTEndpointTypeDef *)((uint32_t)USB_OTG_HS + USB_OTG_OUT_ENDPOINT_BASE
                                          + (EP_OUT & 0x0Fu) * USB_OTG_EP_REG_SIZE);
}

/* Re-arm the isochronous OUT endpoint for the frame after the one the last
 * packet arrived in. The HAL picks the parity from the CURRENT frame number,
 * which is one frame late whenever the host put its packet near the end of
 * a frame and the interrupt ran after the next SOF: the endpoint then waits
 * for frame N+2, the packet of N+1 is dropped, and nothing reports it (the
 * incomplete-ISO-OUT check skips an endpoint armed for the other parity).
 * DOEPCTL bit 16 (EONUM) holds the parity of the frame the data came in. */
static void arm_out(USBD_HandleTypeDef *pdev, int rx_parity)
{
    USBD_LL_PrepareReceive(pdev, EP_OUT, s_out_pkt, UAC_PKT_MAX);
    if (rx_parity < 0) return;                       /* no packet: HAL's pick */
    USB_OTG_OUTEndpointTypeDef *ep = out_ep_regs();
    const uint32_t armed_odd = (ep->DOEPCTL >> 16) & 1u;
    const uint32_t want_odd  = rx_parity ? 0u : 1u;  /* the next frame        */
    if (armed_odd != want_odd)
    {
        ep->DOEPCTL |= want_odd ? USB_OTG_DOEPCTL_SODDFRM : USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
        s_info.out_parity_fixes++;
    }
}

/* ---- class callbacks -------------------------------------------------------- */

static void send_in(USBD_HandleTypeDef *pdev);

static uint8_t c_init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    (void)pdev;
    (void)cfgidx;
    return (uint8_t)USBD_OK;
}

static uint8_t c_deinit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    (void)cfgidx;
    out_stream(pdev, 0u);
    in_stream(pdev, 0u);
    return (uint8_t)USBD_OK;
}

static uint8_t c_setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    static uint8_t  alt_reply;
    static uint16_t status_reply;
    const uint8_t   recipient = req->bmRequest & 0x1Fu;

    switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
    case USB_REQ_TYPE_STANDARD:
        switch (req->bRequest)
        {
        case USB_REQ_GET_STATUS:
            status_reply = 0u;
            USBD_CtlSendData(pdev, (uint8_t *)&status_reply, 2u);
            return (uint8_t)USBD_OK;
        case USB_REQ_GET_INTERFACE:
        {
            const uint8_t itf = LOBYTE(req->wIndex);
            alt_reply = itf == ITF_OUT ? s_out_on : (itf == ITF_IN ? s_in_on : 0u);
            USBD_CtlSendData(pdev, &alt_reply, 1u);
            return (uint8_t)USBD_OK;
        }
        case USB_REQ_SET_INTERFACE:
        {
            const uint8_t itf = LOBYTE(req->wIndex);
            const uint8_t alt = LOBYTE(req->wValue);
            if (pdev->dev_state != USBD_STATE_CONFIGURED || alt > 1u
                || (itf == ITF_AC && alt != 0u) || itf > ITF_IN)
            {
                USBD_CtlError(pdev, req);
                return (uint8_t)USBD_FAIL;
            }
            if (itf == ITF_OUT) out_stream(pdev, alt);
            if (itf == ITF_IN)  in_stream(pdev, alt);
            return (uint8_t)USBD_OK;   /* the core sends the status stage */
        }
        case USB_REQ_CLEAR_FEATURE:
            return (uint8_t)USBD_OK;
        default:
            USBD_CtlError(pdev, req);
            return (uint8_t)USBD_FAIL;
        }

    case USB_REQ_TYPE_CLASS:
        /* No units, no controls. A host may still ask about the sampling
         * frequency (CS 0x01 on an endpoint): answer 48 kHz, accept and
         * ignore anything it sets, so nothing stalls. */
        if (req->bmRequest & 0x80u)
        {
            const uint16_t n = req->wLength < sizeof s_ctl ? req->wLength : sizeof s_ctl;
            memset(s_ctl, 0, sizeof s_ctl);
            if (HIBYTE(req->wValue) == 0x01u && recipient == 0x02u)
            {
                s_ctl[0] = LO(UAC_RATE);
                s_ctl[1] = HI(UAC_RATE);
                s_ctl[2] = (uint8_t)(UAC_RATE >> 16);
            }
            USBD_CtlSendData(pdev, s_ctl, n);
        }
        else if (req->wLength)
        {
            const uint16_t n = req->wLength < sizeof s_ctl ? req->wLength : sizeof s_ctl;
            USBD_CtlPrepareRx(pdev, s_ctl, n);
        }
        else if (recipient == 0x02u)
        {
            USBD_CtlSendStatus(pdev);  /* an interface request gets it from the core */
        }
        return (uint8_t)USBD_OK;

    case USB_REQ_TYPE_VENDOR:
        /* Reboot into the bootloader (DFU), so a build can be flashed
         * without a power-cycle: there is no HostLink in this mode. */
        if (recipient == 0x00u && req->bRequest == UAC_VENDOR_REBOOT_DFU
            && req->wValue == UAC_VENDOR_REBOOT_MAGIC && req->wLength == 0u)
        {
            s_reboot_req = 1u;
            USBD_CtlSendStatus(pdev);
            return (uint8_t)USBD_OK;
        }
        if (recipient == 0x00u && req->bRequest == UAC_VENDOR_INFO
            && (req->bmRequest & 0x80u))
        {
            static UAC_Info snap;
            snap = UAC_GetInfo();
            const uint16_t n = req->wLength < sizeof snap ? req->wLength : (uint16_t)sizeof snap;
            USBD_CtlSendData(pdev, (uint8_t *)&snap, n);
            return (uint8_t)USBD_OK;
        }
        USBD_CtlError(pdev, req);
        return (uint8_t)USBD_FAIL;

    default:
        USBD_CtlError(pdev, req);
        return (uint8_t)USBD_FAIL;
    }
}

static uint8_t c_ep0_rx_ready(USBD_HandleTypeDef *pdev)
{
    (void)pdev;
    return (uint8_t)USBD_OK;
}

static uint8_t c_data_in(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    (void)pdev;
    if (epnum == (EP_IN & 0x7Fu))
    {
        s_in_busy = 0u;
        s_info.in_packets++;
        /* Queue the next one now: from here the parity is for the next
         * frame. Waiting for SOF would skip every other frame. */
        if (s_in_on) send_in(pdev);
    }
    return (uint8_t)USBD_OK;
}

static uint8_t c_data_out(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    if (epnum != EP_OUT || !s_out_on) return (uint8_t)USBD_OK;

    const int      rxpar  = (int)((out_ep_regs()->DOEPCTL >> 16) & 1u);
    const uint32_t bytes  = USBD_LL_GetRxDataSize(pdev, epnum);
    const uint32_t frames = bytes / UAC_FRAME_BYTES;
    uint32_t       wr     = s_out_wr;
    const uint32_t rd     = s_out_rd;
    const uint8_t *p      = s_out_pkt;
    /* Too full (the host sent a burst after a stall): drop this packet
     * rather than carry the latency, for the same reason as the IN side. */
    if (s_out_primed && s_out_avg > OUT_TARGET + 128.0f && wr - rd > (uint32_t)OUT_TARGET + 128u)
    {
        s_out_avg = OUT_TARGET + 64.0f;
        s_info.out_resyncs++;
        arm_out(pdev, rxpar);
        return (uint8_t)USBD_OK;
    }
    for (uint32_t i = 0; i < frames; i++, p += UAC_FRAME_BYTES)
    {
        if (wr - rd >= RING) { s_info.out_overruns++; break; }
        s_out_ring[wr & RING_MASK][0] = get24(p);
        s_out_ring[wr & RING_MASK][1] = get24(p + 3);
        wr++;
    }
    __DMB();
    s_out_wr = wr;
    s_info.out_packets++;
    arm_out(pdev, rxpar);
    return (uint8_t)USBD_OK;
}

static void send_in(USBD_HandleTypeDef *pdev)
{
    uint32_t       rd   = s_in_rd;
    const uint32_t fill = s_in_wr - rd;

    if (!s_in_primed && fill >= IN_PRIME) s_in_primed = 1u;

    /* A big excess (a start-up overshoot, or packets the host skipped) is
     * dropped in one go: one small glitch. Draining it by sending faster
     * would tell macOS the device runs fast -- it clocks the device from
     * these packet sizes -- and its model would drift until it resyncs. */
    if (s_in_primed && s_in_avg > IN_TARGET + 96.0f)
    {
        const uint32_t f = s_in_wr - rd;
        if (f > (uint32_t)IN_TARGET)
        {
            rd += f - (uint32_t)IN_TARGET;
            s_in_avg = IN_TARGET;
            s_info.in_resyncs++;
        }
    }

    /* Frames this millisecond: exactly the rate the feedback reports. */
    s_in_frac += (float)s_fbq * (1.0f / 16384.0f);
    uint32_t n = (uint32_t)s_in_frac;
    s_in_frac -= (float)n;
    if (n < UAC_NOM_FRAMES - 1u) n = UAC_NOM_FRAMES - 1u;
    if (n > UAC_MAX_FRAMES)      n = UAC_MAX_FRAMES;

    uint8_t *p = s_in_pkt;
    for (uint32_t i = 0; i < n; i++, p += UAC_FRAME_BYTES)
    {
        if (s_in_primed && s_in_wr - rd > 0u)
        {
            put24(p, s_in_ring[rd & RING_MASK][0]);
            put24(p + 3, s_in_ring[rd & RING_MASK][1]);
            rd++;
        }
        else
        {
            if (s_in_primed) { s_info.in_underruns++; s_in_primed = 0u; }
            memset(p, 0, UAC_FRAME_BYTES);
        }
    }
    s_in_rd   = rd;
    s_in_busy = 1u;
    s_in_len  = (uint16_t)(n * UAC_FRAME_BYTES);
    USBD_LL_Transmit(pdev, EP_IN, s_in_pkt, s_in_len);
}

static uint32_t s_last_sof_cyc, s_last_cb_cyc;

static uint8_t c_sof(USBD_HandleTypeDef *pdev)
{
    const uint32_t sofs = ++s_info.sofs;
    {
        /* Interrupt-latency watch: a stalled IRQ shows as a long gap. */
        const uint32_t cyc = DWT->CYCCNT;
        if (s_last_sof_cyc && cyc - s_last_sof_cyc > s_info.sof_gap_max)
        {
            s_info.sof_gap_max = cyc - s_last_sof_cyc;
            s_info.sof_gap_at  = sofs;
        }
        s_last_sof_cyc = cyc;
    }

    if ((sofs & 1023u) == 0u)
    {
        const uint32_t cyc = DWT->CYCCNT;
        if (s_sof_mark_cyc)
        {
            const float c = (float)(cyc - s_sof_mark_cyc) / 1024.0f;
            s_cyc_per_sof = s_cyc_per_sof > 0.0f ? s_cyc_per_sof + (c - s_cyc_per_sof) * 0.25f : c;
        }
        s_sof_mark_cyc = cyc ? cyc : 1u;
        if (s_cyc_per_sof > 0.0f && s_cyc_per_frame > 0.0f)
        {
            const float r = s_cyc_per_sof / s_cyc_per_frame;
            if (r > 47.9f && r < 48.1f)                /* else: something is off */
            {
                s_rate = r;
                pll3_steer();
            }
        }
        s_info.rate = s_rate;
    }

    if (s_out_on)
    {
        s_out_avg += ((float)(s_out_wr - s_out_rd) - s_out_avg) * (1.0f / 256.0f);
        s_info.out_fill = s_out_avg;
        s_info.feedback = s_fbq;   /* what the host is assumed to send */
    }
    if (s_in_on)
    {
        s_in_avg += ((float)(s_in_wr - s_in_rd) - s_in_avg) * (1.0f / 256.0f);
        s_info.in_fill = s_in_avg;
        if (!s_in_busy) send_in(pdev);
    }
    return (uint8_t)USBD_OK;
}

static uint8_t c_iso_in_incomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    /* The HAL aborted and disabled the endpoint (the host did not take the
     * packet in its frame). Offer the SAME packet again at once: its frames
     * already left the ring, and building a new one would drop them and
     * drain the ring by a packet for every miss. */
    if (epnum != (EP_IN & 0x7Fu)) return (uint8_t)USBD_OK;
    s_info.in_incomplete++;
    s_in_busy = 0u;
    if (s_in_on && s_in_len)
    {
        s_in_busy = 1u;
        USBD_LL_Transmit(pdev, EP_IN, s_in_pkt, s_in_len);
    }
    return (uint8_t)USBD_OK;
}

static uint8_t c_iso_out_incomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    s_info.out_incomplete++;
    if (epnum == EP_OUT && s_out_on)
        USBD_LL_PrepareReceive(pdev, EP_OUT, s_out_pkt, UAC_PKT_MAX);
    return (uint8_t)USBD_OK;
}

static uint8_t *c_cfg(uint16_t *len)
{
    *len = (uint16_t)sizeof k_cfg;
    return (uint8_t *)(uintptr_t)k_cfg;
}

static USBD_ClassTypeDef s_class = {
    .Init                          = c_init,
    .DeInit                        = c_deinit,
    .Setup                         = c_setup,
    .EP0_TxSent                    = NULL,
    .EP0_RxReady                   = c_ep0_rx_ready,
    .DataIn                        = c_data_in,
    .DataOut                       = c_data_out,
    .SOF                           = c_sof,
    .IsoINIncomplete               = c_iso_in_incomplete,
    .IsoOUTIncomplete              = c_iso_out_incomplete,
    .GetHSConfigDescriptor         = c_cfg,
    .GetFSConfigDescriptor         = c_cfg,
    .GetOtherSpeedConfigDescriptor = c_cfg,
    .GetDeviceQualifierDescriptor  = NULL,
};

/* ---- public ----------------------------------------------------------------- */

void UAC_Start(const char *product)
{
    if (s_started) return;
    if (product) s_product = product;

    /* The codec's clock becomes steerable (before the SAI starts). */
    pll3_relock();

    /* The cycle counter times the codec against USB (the rate estimate). */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* libDaisy's USBD_LL_Init brings up the HS core in FS mode on PB14/15
     * (pins, clock, IRQs) and sizes the FIFOs for CDC. */
    USBD_Init(&s_dev, &s_desc, 1 /* DEVICE_HS */);

    /* FIFOs in 32-bit words, set in order (each offset follows the last):
     * RX holds two 294-byte OUT packets plus setup, TX0 control, TX1 the
     * 3-byte feedback, TX2 two 294-byte IN packets. 592 of 1024 words. */
    HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_HS, 0x100);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS, 0, 0x40);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS, 1, 0x10);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS, 2, 0x100);

    /* Start-of-frame drives the IN packets, the feedback and the rate
     * measurement; libDaisy leaves it masked. */
    USB_OTG_HS->GINTMSK |= USB_OTG_GINTMSK_SOFM;

    USBD_RegisterClass(&s_dev, &s_class);
    USBD_Start(&s_dev);
    HAL_PWREx_EnableUSBVoltageDetector();
    s_started = 1u;
}

void UAC_Process(const float *const *in, float **out, size_t frames)
{
    const uint32_t total = s_codec_frames + (uint32_t)frames;
    s_codec_frames = total;
    {
        const uint32_t cyc = DWT->CYCCNT;
        if (s_last_cb_cyc && cyc - s_last_cb_cyc > s_info.cb_gap_max)
        {
            s_info.cb_gap_max = cyc - s_last_cb_cyc;
            s_info.cb_gap_at  = s_info.sofs;
        }
        s_last_cb_cyc = cyc;
    }
    if (total - s_cf_mark_frames >= 65536u)
    {
        const uint32_t cyc = DWT->CYCCNT;
        if (s_cf_mark_cyc)
        {
            const float c = (float)(cyc - s_cf_mark_cyc) / (float)(total - s_cf_mark_frames);
            s_cyc_per_frame = s_cyc_per_frame > 0.0f
                                  ? s_cyc_per_frame + (c - s_cyc_per_frame) * 0.25f : c;
        }
        s_cf_mark_cyc    = cyc ? cyc : 1u;
        s_cf_mark_frames = total;
    }

    /* computer -> J9/J10 */
    if (s_out_on)
    {
        uint32_t       rd = s_out_rd;
        const uint32_t wr = s_out_wr;
        __DMB();
        if (!s_out_primed && wr - rd >= OUT_PRIME) s_out_primed = 1u;
        for (size_t i = 0; i < frames; i++)
        {
            if (s_out_primed && wr - rd > 0u)
            {
                out[0][i] = (float)s_out_ring[rd & RING_MASK][0] * (1.0f / 8388608.0f);
                out[1][i] = (float)s_out_ring[rd & RING_MASK][1] * (1.0f / 8388608.0f);
                rd++;
            }
            else
            {
                if (s_out_primed) { s_info.out_underruns++; s_out_primed = 0u; }
                out[0][i] = out[1][i] = 0.0f;
            }
        }
        s_out_rd = rd;
    }
    else
    {
        for (size_t i = 0; i < frames; i++) out[0][i] = out[1][i] = 0.0f;
    }

    /* J1/J2 -> computer */
    if (s_in_on)
    {
        uint32_t       wr = s_in_wr;
        const uint32_t rd = s_in_rd;
        for (size_t i = 0; i < frames; i++)
        {
            if (wr - rd >= RING) { s_info.in_overruns++; break; }
            s_in_ring[wr & RING_MASK][0] = f2s24(in[0][i]);
            s_in_ring[wr & RING_MASK][1] = f2s24(in[1][i]);
            wr++;
        }
        __DMB();
        s_in_wr = wr;
    }
}

uint8_t UAC_RebootRequested(void)
{
    return s_reboot_req;
}

uint8_t UAC_State(void)
{
    if (!s_started) return UAC_OFF;
    if (s_dev.dev_state != USBD_STATE_CONFIGURED) return UAC_WAITING;
    return (s_out_on || s_in_on) ? UAC_STREAMING : UAC_IDLE;
}

UAC_Info UAC_GetInfo(void)
{
    UAC_Info i;
    memcpy(&i, (const void *)&s_info, sizeof i);
    i.state  = UAC_State();
    i.cyc_per_frame = s_cyc_per_frame;
    i.pll_frac      = s_pll_frac;
    i.pll_err_ppm   = s_pll_err_ppm;
    i.cyc_per_sof   = s_cyc_per_sof;
    i.out_raw       = s_out_wr - s_out_rd;
    i.in_raw        = s_in_wr - s_in_rd;
    i.out_on = s_out_on;
    i.in_on  = s_in_on;
    return i;
}

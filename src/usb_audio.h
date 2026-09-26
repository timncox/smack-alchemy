/*
 * usb_audio -- the Alchemy Lab as a class-compliant USB audio interface on
 * the front USB-C: 2 channels out of the computer (to the codec outs, J9/J10)
 * and 2 channels into it (from the codec ins, J1/J2), 48 kHz, 24-bit.
 *
 * USB Audio Class 1, full speed, no driver on macOS/iOS/Windows/Linux.
 *
 *   interface 0  AudioControl: USB-in -> line-out, line-in -> USB-out; no
 *                volume/mute units, so the computer cannot scale the signal
 *                (J9/J10 are DC-coupled: a DAW can send CV through them)
 *   interface 1  AudioStreaming OUT, EP 0x01 isochronous ADAPTIVE
 *   interface 2  AudioStreaming IN,  EP 0x82 isochronous SYNCHRONOUS
 *
 * The HOST is the clock. The codec's clock (PLL3, which libDaisy leaves
 * 250 ppm fast) is steered once a second so the codec makes exactly 48.000
 * frames per USB frame: its rate is measured against the CPU cycle counter
 * (a few ppm), and the ring fills add a trim of at most 20 ppm. Every
 * packet both ways is 48 frames. Lock-free single-producer/single-consumer
 * rings sit between the USB interrupt and the audio callback.
 *
 * Why not asynchronous with a feedback endpoint (the first design, and the
 * textbook one): macOS's usbaudiod (2026-09) models a device at its nominal
 * rate whatever the feedback says, and a few missed feedback polls a second
 * made it restart the whole stream every ~30 s. Both are logged by
 * usbaudiod ("ioDriftNS", "enqueueAvailableFeedbackTransferBlocks_error").
 *
 * One role per boot on the front port: never together with HostLink or the
 * Launchpad host. Uses libDaisy's USBD_LL_Init for the HS core in FS mode
 * (PB14/PB15), then re-splits its FIFOs for the isochronous endpoints.
 * Needs usbd_ctlreq_uac.c (ST's usbd_ctlreq.c allowing 3 interfaces).
 *
 * Shared by copy with smack-alchemy and belt-alchemy.
 */
#ifndef USB_AUDIO_H
#define USB_AUDIO_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bring the device up on the front USB-C. Call once, from main(). */
void UAC_Start(const char *product);

/* From the audio callback: in = the codec inputs (to the computer),
 * out = the codec outputs (from the computer). Non-interleaved, 2 channels.
 * Writes silence to out while the computer is not playing. */
void UAC_Process(const float *const *in, float **out, size_t frames);

enum
{
    UAC_OFF = 0,        /* not started                               */
    UAC_WAITING,        /* started, not configured by a host yet     */
    UAC_IDLE,           /* configured, no stream open                */
    UAC_STREAMING       /* at least one direction streaming          */
};
uint8_t UAC_State(void);

/* A host asked for the bootloader: vendor request to the device,
 * bRequest UAC_VENDOR_REBOOT_DFU, wValue UAC_VENDOR_REBOOT_MAGIC, no data.
 * The app polls this and resets (after the status stage has gone out). */
#define UAC_VENDOR_REBOOT_DFU   0xB0u
#define UAC_VENDOR_REBOOT_MAGIC 0xB007u
/* Vendor IN request 0xB1 to the device returns UAC_Info (little-endian,
 * as laid out here), for a live readout while streaming. */
#define UAC_VENDOR_INFO         0xB1u
uint8_t UAC_RebootRequested(void);

/* For a diagnostic report. */
typedef struct
{
    uint8_t  state;
    uint8_t  out_on, in_on;        /* alt setting of interface 1 / 2      */
    uint32_t sofs;                 /* SOFs seen                           */
    uint32_t out_packets, in_packets;
    uint32_t out_underruns;        /* audio callback found the ring empty */
    uint32_t out_overruns;         /* USB found the ring full             */
    uint32_t in_underruns;         /* a packet had to be padded           */
    uint32_t in_overruns;          /* the audio callback found it full    */
    uint32_t out_incomplete, in_incomplete, fb_incomplete;
    uint32_t feedback;             /* the rate packets carry, 10.14       */
    float    rate;                 /* measured codec frames per ms        */
    float    out_fill, in_fill;    /* smoothed ring fill, frames          */
    float    cyc_per_frame;        /* codec, CPU cycles per frame         */
    float    cyc_per_sof;          /* USB, CPU cycles per 1 ms frame      */
    uint32_t out_raw, in_raw;      /* instantaneous ring fill             */
    uint32_t out_resyncs, in_resyncs; /* excess dropped in one go         */
    uint32_t sof_gap_max, sof_gap_at; /* longest SOF-to-SOF, cycles; when */
    uint32_t cb_gap_max, cb_gap_at;   /* longest callback gap; when (SOF#)*/
    uint32_t out_parity_fixes;     /* OUT re-armed for the right frame    */
    uint32_t pll_frac;             /* PLL3 fraction steering the codec    */
    float    pll_err_ppm;          /* codec rate error, ppm (after trim)  */
} UAC_Info;
UAC_Info UAC_GetInfo(void);

#ifdef __cplusplus
}
#endif

#endif

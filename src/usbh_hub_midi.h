/*
 * usbh_hub_midi -- one USB-MIDI device behind one hub, for the ST USB Host
 * Library as libDaisy ships it (which has no hub class).
 *
 * Why: USB-C "OTG adapters with power" usually have a hub inside (seen
 * 2026-09-25: VIA Labs 2109:2817 in front of a Launchpad Mini MK3), and the
 * Alchemy Lab's front port has no VBUS, so a powered adapter is how a
 * bus-powered controller gets 5 V.
 *
 * Scope, on purpose:
 *   - one hub level, the first MIDI-streaming device found on its ports;
 *   - full-speed only. The Lab's host port runs at full speed, so a USB 2.0
 *     hub runs its upstream at full speed and passes full-speed packets
 *     through unchanged: no split transactions. Low-speed devices (which
 *     would need PRE packets) are refused;
 *   - the hub stays at address 1 (the core enumerates it), the device is
 *     given address 2 here. After the device is configured the hub is not
 *     polled again; an unplug surfaces as transfer errors / a host reset.
 *
 * It registers as the class for bInterfaceClass 0x09, enumerates the device
 * behind the hub itself with standard control requests (re-pointing the
 * core's control pipes at address 0, then 2), opens the device's bulk MIDI
 * endpoints and runs the same receive loop as usbh_midi.c.
 */
#ifndef USBH_HUB_MIDI_H
#define USBH_HUB_MIDI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbh_core.h"

extern USBH_ClassTypeDef USBH_hub_midi;
#define USBH_HUB_MIDI_CLASS (&USBH_hub_midi)

typedef void (*HUBMIDI_RxCallback)(uint8_t *buf, size_t len, void *user);

/* 1 once the device behind the hub is configured and its pipes are open. */
uint8_t HUBMIDI_Ready(USBH_HandleTypeDef *phost);

/* Non-blocking: USBH_BUSY while the previous transfer is in flight, USBH_OK
 * when queued, USBH_FAIL when not ready. len <= HUBMIDI_OutSize(). */
USBH_StatusTypeDef HUBMIDI_Transmit(USBH_HandleTypeDef *phost, uint8_t *data, uint16_t len);
uint16_t HUBMIDI_OutSize(USBH_HandleTypeDef *phost);

void HUBMIDI_SetReceiveCallback(HUBMIDI_RxCallback cb, void *user);

/* For a diagnostic report. */
typedef struct
{
    uint8_t  state;        /* internal state number */
    uint8_t  ports;        /* the hub's port count */
    uint8_t  port;         /* the port the device is on, 0 = none yet */
    uint16_t port_status;  /* last wPortStatus read */
    uint16_t vid, pid;     /* the device behind the hub */
    uint8_t  fail_state;   /* the state that failed, 0 = none */
    uint8_t  fail_code;    /* USBH_StatusTypeDef of that failure */
} HUBMIDI_Info;
HUBMIDI_Info HUBMIDI_GetInfo(void);

#ifdef __cplusplus
}
#endif

#endif

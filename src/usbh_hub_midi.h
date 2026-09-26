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
 *   - one hub level, up to HUBMIDI_MAX_DEVICES MIDI-streaming devices on its
 *     ports; free ports are rescanned every 500 ms, so one can be plugged in
 *     later;
 *   - full-speed only. The Lab's host port runs at full speed, so a USB 2.0
 *     hub runs its upstream at full speed and passes full-speed packets
 *     through unchanged: no split transactions. Low-speed devices (which
 *     would need PRE packets) are refused;
 *   - besides MIDI, an XInput gamepad is taken (interrupt IN, polled every
 *     bInterval ms); nothing is ever sent to it;
 *   - the hub stays at address 1 (the core enumerates it); devices behind
 *     it get addresses 2, 3, ... here. A device that is not MIDI (many
 *     dongles carry an Ethernet chip on a port, e.g. Realtek 0bda:8153) or
 *     will not enumerate has its port switched off and is skipped.
 *     Configured ports are not polled again; an unplug surfaces as transfer errors / a host reset.
 *
 * It registers as the class for bInterfaceClass 0x09, enumerates the device
 * behind the hub itself with standard control requests (re-pointing the
 * core's control pipes at address 0, then the new address), opens the device's bulk MIDI
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

/* Controllers served at once (e.g. a Launchpad Mini, a Launch Control and
 * a gamepad). */
#define HUBMIDI_MAX_DEVICES 3

/* What a slot carries. An XInput gamepad (vendor interface ff/5d/01, e.g.
 * a GP2040-CE controller in XInput mode, 045E:028E) delivers its 20-byte
 * input reports through the same receive callback. */
#define HUBMIDI_KIND_MIDI   0
#define HUBMIDI_KIND_XINPUT 1

typedef void (*HUBMIDI_RxCallback)(uint8_t dev, uint8_t *buf, size_t len, void *user);

/* 1 once any device behind the hub is configured. */
uint8_t HUBMIDI_Ready(USBH_HandleTypeDef *phost);
/* Per device slot 0..HUBMIDI_MAX_DEVICES-1. */
uint8_t HUBMIDI_DevReady(USBH_HandleTypeDef *phost, uint8_t dev);
void    HUBMIDI_DevId(uint8_t dev, uint16_t *vid, uint16_t *pid);
uint8_t HUBMIDI_DevKind(uint8_t dev);

/* Non-blocking: USBH_BUSY while the previous transfer to that device is in
 * flight, USBH_OK when queued, USBH_FAIL when not ready. */
USBH_StatusTypeDef HUBMIDI_Transmit(USBH_HandleTypeDef *phost, uint8_t dev, uint8_t *data,
                                    uint16_t len);
uint16_t HUBMIDI_OutSize(USBH_HandleTypeDef *phost, uint8_t dev);

void HUBMIDI_SetReceiveCallback(HUBMIDI_RxCallback cb, void *user);

/* For a diagnostic report. */
typedef struct
{
    uint8_t  state;        /* enumeration state number (see the .c) */
    uint8_t  ports;        /* the hub's port count */
    uint8_t  port;         /* the port being scanned */
    uint16_t port_status;  /* last wPortStatus read */
    uint8_t  fail_state;   /* the last state that failed, 0 = none */
    uint8_t  fail_code;    /* USBH_StatusTypeDef of that failure */
    uint16_t skipped;      /* ports rejected (bit n = port n) */
    uint16_t done;         /* ports carrying a configured device */
    uint16_t dev_vid[HUBMIDI_MAX_DEVICES], dev_pid[HUBMIDI_MAX_DEVICES];
    uint8_t  dev_port[HUBMIDI_MAX_DEVICES]; /* 0 = slot empty */
    uint8_t  dev_kind[HUBMIDI_MAX_DEVICES];
    /* receive side per slot: transfers armed, completed with data, NAKed
     * (interrupt: nothing to say), failed (error / stall) */
    uint32_t rx_arms[HUBMIDI_MAX_DEVICES], rx_done[HUBMIDI_MAX_DEVICES];
    uint32_t rx_nak[HUBMIDI_MAX_DEVICES], rx_err[HUBMIDI_MAX_DEVICES];
    uint32_t rx_stale[HUBMIDI_MAX_DEVICES]; /* XInput: re-armed while still pending */
} HUBMIDI_Info;
HUBMIDI_Info HUBMIDI_GetInfo(void);

#ifdef __cplusplus
}
#endif

#endif

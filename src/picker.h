/*
 * picker — swap firmware from the SD card without a computer.
 *
 * A settings page (B2+B3 held two seconds opens Settings; B1 cycles its
 * pages) with three pots:
 *
 *   pot 0  FILE   choose among the .bin files in /alchemy on the card.
 *                 The ring shows one dot per file, the chosen one bright.
 *   pot 1  FLASH  turn below ~85 % and then past ~95 % to write the chosen
 *                 file into the QSPI app slot, verify it, and reboot into
 *                 it. The dip-then-sweep is the arming gesture: a pot that
 *                 happens to be parked at maximum cannot fire on entry.
 *   pot 2  DFU    same gesture: reboot into the bootloader's update mode
 *                 and stay there. The escape hatch for a firmware that has
 *                 no picker of its own.
 *
 * Every Alchemy firmware that includes this page can hand the module to any
 * other, so the card becomes the module's library. The write is exactly what
 * a DFU upload writes -- the .bin bytes at 0x90040000 -- so the bootloader
 * cannot tell the difference. It refuses images that are not bootable from
 * SRAM (an internal-flash build would leave the bootloader with nothing to
 * boot) and never touches the calibration or preset sectors above the slot.
 *
 * The work is spread over control-loop frames (one 4 KB chunk per frame, one
 * 64 KB erase per frame) so the rings keep animating and HostLink keeps
 * answering; the audio callback runs throughout, from SRAM.
 *
 * Files get onto the card by pulling it, or over USB through HostLink's
 * filesystem block (the web programmer's file panel) when the firmware
 * exposes it -- this one does.
 */
#pragma once

#include <cstdint>

namespace alchemy {
class Settings;
class SdCard;
#if defined(ALCHEMY_BOARD_V2)
class AlchemyLabV2;
using AlchemyLab = AlchemyLabV2;
#else
class AlchemyLabV1;
using AlchemyLab = AlchemyLabV1;
#endif
}

namespace picker {

/* Folder on the card, FatFS form (volume "0:"). */
constexpr const char* kDir = "0:/alchemy";

/* Declare the page's three controls. Call once, after sd.Init() and before
 * presets.Init(); the card is not touched until the page is opened. */
void Install(alchemy::Settings&   settings,
             uint8_t              page,
             alchemy::SdCard&     sd,
             alchemy::AlchemyLab& hw);

/* True while an erase / write / verify is in flight (the main loop can hold
 * off autosaves and other flash traffic). */
bool Busy();

} // namespace picker

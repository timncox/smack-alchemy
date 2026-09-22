/*
 * picker — implementation. See picker.h for the why.
 *
 * Memory: SDMMC1 reads by DMA, and DMA cannot reach DTCM, which is where a
 * bootloader build's .bss lives. Everything FatFS reads into (the FIL, the
 * DIR, the chunk buffer) is therefore in `.axi_bss` -- AXI SRAM, added by
 * src/axi_bss.ld -- per alchemy/storage/sd_card.h's three placement rules.
 * That section is NOLOAD, so every object here is cleared explicitly.
 *
 * Chunk reads are 4 KB at 4 KB offsets, so FatFS's direct-sector transfers
 * always land word-aligned (rule 3); the 8-byte header read goes through
 * FatFS's own sector window and is exempt.
 */
#include "picker.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "daisy_seed.h"
#include "ff.h"

#include "alchemy/hw/alchemy_lab.h"
#include "alchemy/hw/v2_calibration.h"
#include "alchemy/storage/sd_card.h"
#include "alchemy/surface/settings.h"

using namespace alchemy;

namespace {

constexpr uint32_t kAppSlot   = 0x90040000u;                 /* QSPIFLASH ORIGIN in the SDK linker */
constexpr uint32_t kAppMax    = kV2CalQspiAddr - kAppSlot;   /* stop short of the cal + preset sectors */
constexpr uint32_t kChunk     = 4096u;
constexpr uint32_t kEraseStep = 65536u;

/* Where a bootloader image may legitimately point its stack and reset. */
constexpr uint32_t kDtcmLo   = 0x20000000u, kDtcmHi   = 0x20020000u;
constexpr uint32_t kAxiLo    = 0x24000000u, kAxiHi    = 0x24080000u;
constexpr uint32_t kQspiHi   = 0x90000000u + 8u * 1024u * 1024u;
constexpr uint32_t kIflashLo = 0x08000000u, kIflashHi = 0x08020000u;

constexpr int kMaxFiles = 16;
constexpr int kNameLen  = 32;

/* Arming gesture thresholds (pot 0..1). */
constexpr float kArmBelow  = 0.85f;
constexpr float kFireAbove = 0.95f;

enum class St : uint8_t
{
    Unscanned,  /* page not opened yet, or a rescan is due */
    NoCard,
    NoDir,
    Empty,
    Listed,
    Comparing,  /* reading the file against the running image, before any erase */
    Same,       /* it IS the running image: nothing written, no reboot */
    Erasing,
    Writing,
    Verifying,
    Done,
    Error,
};

struct State
{
    St          st = St::Unscanned;
    SdCard*     sd = nullptr;
    AlchemyLab* hw = nullptr;

    int      count = 0;
    char     name[kMaxFiles][kNameLen] = {};
    uint32_t size[kMaxFiles] = {};
    int      sel = 0;

    bool     flash_armed = false;   /* pot 1 seen below kArmBelow since entry */
    bool     dfu_armed   = false;   /* pot 2 likewise */
    uint32_t last_tick   = 0;       /* to notice the page being re-entered */

    uint32_t img_size = 0;
    uint32_t off      = 0;          /* write / verify offset */
    uint32_t erased   = 0;          /* erase offset */
    float    progress = 0.0f;       /* 0..1 for the FLASH ring */
    const char* err   = "";
};

State g;

/* FatFS-touched objects: DMA-reachable, 32-byte aligned, zeroed before use. */
alignas(32) ALCHEMY_SDMMC_BSS FIL     s_fil;
alignas(32) ALCHEMY_SDMMC_BSS DIR     s_dir;
alignas(32) ALCHEMY_SDMMC_BSS FILINFO s_fno;
alignas(32) ALCHEMY_SDMMC_BSS uint8_t s_chunk[kChunk + 32u];

/* ---- colours ------------------------------------------------------------ */

constexpr LedPanel::Rgb kFileOn  = {0x40, 0xFF, 0x60};
constexpr LedPanel::Rgb kFileOff = {0x10, 0x18, 0x10};
constexpr LedPanel::Rgb kNoCard  = {0x80, 0x00, 0x00};
constexpr LedPanel::Rgb kNoDir   = {0x80, 0x30, 0x00};
constexpr LedPanel::Rgb kArm     = {0xFF, 0x60, 0x00};
constexpr LedPanel::Rgb kErase   = {0xFF, 0xA0, 0x00};
constexpr LedPanel::Rgb kWrite   = {0x20, 0x60, 0xFF};
constexpr LedPanel::Rgb kVerify  = {0x20, 0xFF, 0x60};
constexpr LedPanel::Rgb kCompare = {0x20, 0xC0, 0xC0};
constexpr LedPanel::Rgb kSame    = {0x40, 0xFF, 0x60};
constexpr LedPanel::Rgb kDone    = {0xFF, 0xFF, 0xFF};
constexpr LedPanel::Rgb kError   = {0xFF, 0x00, 0x00};
constexpr LedPanel::Rgb kDfu     = {0xC0, 0x20, 0xFF};

/* ---- card ---------------------------------------------------------------- */

bool ready(uint32_t now)
{
    return g.sd && g.sd->EnsureMounted(now);
}

void scan(uint32_t now)
{
    g.count = 0;
    g.sel   = 0;
    if (!ready(now)) { g.st = St::NoCard; return; }

    SdCard::BusyGuard guard(*g.sd);
    memset(&s_dir, 0, sizeof s_dir);
    if (f_opendir(&s_dir, picker::kDir) != FR_OK) { g.st = St::NoDir; return; }

    for (;;)
    {
        memset(&s_fno, 0, sizeof s_fno);
        if (f_readdir(&s_dir, &s_fno) != FR_OK || s_fno.fname[0] == '\0') break;
        if (s_fno.fattrib & (AM_DIR | AM_HID)) continue;
        /* macOS writes an AppleDouble "._name.bin" beside every file it
         * copies to FAT; those are not firmware. */
        if (s_fno.fname[0] == '.') continue;
        const char* fn = s_fno.fname;
        size_t      L  = strlen(fn);
        if (L < 5 || strcasecmp(fn + L - 4, ".bin") != 0) continue;
        if (L > (size_t)kNameLen - 1) continue;   /* too long to reopen: skip */
        if (g.count >= kMaxFiles) break;
        memcpy(g.name[g.count], fn, L + 1);
        g.size[g.count] = (uint32_t)s_fno.fsize;
        g.count++;
    }
    f_closedir(&s_dir);

    /* Alphabetical, so the ring is stable across cards and rescans. */
    for (int i = 1; i < g.count; i++)
        for (int j = i; j > 0 && strcasecmp(g.name[j - 1], g.name[j]) > 0; j--)
        {
            char     tn[kNameLen];
            uint32_t ts = g.size[j];
            memcpy(tn, g.name[j], kNameLen);
            memcpy(g.name[j], g.name[j - 1], kNameLen);
            memcpy(g.name[j - 1], tn, kNameLen);
            g.size[j]     = g.size[j - 1];
            g.size[j - 1] = ts;
        }

    g.st = g.count ? St::Listed : St::Empty;
}

/* ---- flashing, one step per control frame ------------------------------- */

void fail(const char* why)
{
    f_close(&s_fil);
    g.err = why;
    g.st  = St::Error;
}

void begin_flash(uint32_t now)
{
    if (!ready(now)) { g.st = St::NoCard; return; }

    char path[64];
    snprintf(path, sizeof path, "%s/%s", picker::kDir, g.name[g.sel]);

    SdCard::BusyGuard guard(*g.sd);
    memset(&s_fil, 0, sizeof s_fil);
    if (f_open(&s_fil, path, FA_READ) != FR_OK) { g.err = "open"; g.st = St::Error; return; }

    uint32_t size = (uint32_t)f_size(&s_fil);
    if (size < 512u || size > kAppMax) { fail("size"); return; }

    /* Vector-table sanity, the same test the bootloader applies before it
     * jumps: stack in RAM, reset vector in SRAM or QSPI. An image built for
     * internal flash is refused here instead of poisoning the slot. */
    UINT n = 0;
    if (f_read(&s_fil, s_chunk, 8, &n) != FR_OK || n != 8) { fail("read"); return; }
    uint32_t sp, pc;
    memcpy(&sp, s_chunk, 4);
    memcpy(&pc, s_chunk + 4, 4);
    if (pc >= kIflashLo && pc < kIflashHi) { fail("flash build"); return; }
    const bool sp_ok = (sp >= kDtcmLo && sp <= kDtcmHi) || (sp >= kAxiLo && sp <= kAxiHi);
    const bool pc_ok = (pc >= kAxiLo && pc < kAxiHi) || (pc >= kAppSlot && pc < kQspiHi);
    if (!sp_ok || !pc_ok) { fail("not bootable"); return; }
    f_lseek(&s_fil, 0);

    g.img_size = size;
    g.off      = 0;
    g.erased   = 0;
    g.progress = 0.0f;
    g.st       = St::Comparing;
}

/* Before anything is erased: is the chosen file the image already in the
 * slot? Picking the firmware that is running should cost nothing -- no
 * erase, no write, no reboot, no lost loops. The slot is read through the
 * memory-mapped QSPI view (mapped since DaisySeed::Init for BOOT_SRAM apps,
 * the same view step_verify reads). The first differing chunk hands over to
 * the normal erase / write / verify path from the top of the file. */
void step_compare()
{
    SdCard::BusyGuard guard(*g.sd);
    UINT n = 0;
    if (f_read(&s_fil, s_chunk, kChunk, &n) != FR_OK || n == 0) { fail("read"); return; }

    const uint8_t* mm = (const uint8_t*)(uintptr_t)(kAppSlot + g.off);
    SCB_InvalidateDCache_by_Addr((uint32_t*)(uintptr_t)mm, (int32_t)((n + 31u) & ~31u));
    if (memcmp(mm, s_chunk, n) != 0)
    {
        f_lseek(&s_fil, 0);
        g.off      = 0;
        g.progress = 0.0f;
        g.st       = St::Erasing;
        return;
    }

    g.off     += n;
    g.progress = (float)g.off / (float)g.img_size;
    if (g.off >= g.img_size)
    {
        f_close(&s_fil);
        g.st = St::Same;
    }
}

void step_erase()
{
    daisy::QSPIHandle& q   = g.hw->seed.qspi;
    uint32_t           end = g.erased + kEraseStep;
    if (end > g.img_size) end = g.img_size;
    if (q.Erase(kAppSlot + g.erased, kAppSlot + end) != daisy::QSPIHandle::Result::OK)
    {
        fail("erase");
        return;
    }
    g.erased   = end;
    g.progress = (float)g.erased / (float)g.img_size;
    if (g.erased >= g.img_size)
    {
        g.st       = St::Writing;
        g.progress = 0.0f;
    }
}

void step_write()
{
    SdCard::BusyGuard guard(*g.sd);
    UINT n = 0;
    if (f_read(&s_fil, s_chunk, kChunk, &n) != FR_OK || n == 0) { fail("read"); return; }
    daisy::QSPIHandle& q = g.hw->seed.qspi;
    if (q.Write(kAppSlot + g.off, n, s_chunk) != daisy::QSPIHandle::Result::OK)
    {
        fail("write");
        return;
    }
    g.off     += n;
    g.progress = (float)g.off / (float)g.img_size;
    if (g.off >= g.img_size)
    {
        f_lseek(&s_fil, 0);
        g.off      = 0;
        g.progress = 0.0f;
        g.st       = St::Verifying;
    }
}

void step_verify()
{
    SdCard::BusyGuard guard(*g.sd);
    UINT n = 0;
    if (f_read(&s_fil, s_chunk, kChunk, &n) != FR_OK || n == 0) { fail("read"); return; }

    /* Memory-mapped view; QSPI is back in that mode after Write(). The data
     * cache may hold stale lines for the region, so invalidate first. */
    const uint8_t* mm = (const uint8_t*)(uintptr_t)(kAppSlot + g.off);
    SCB_InvalidateDCache_by_Addr((uint32_t*)(uintptr_t)mm, (int32_t)((n + 31u) & ~31u));
    if (memcmp(mm, s_chunk, n) != 0) { fail("verify"); return; }

    g.off     += n;
    g.progress = (float)g.off / (float)g.img_size;
    if (g.off >= g.img_size)
    {
        f_close(&s_fil);
        g.st = St::Done;
    }
}

/* ---- settings hooks ----------------------------------------------------- */

/* Pot 0: FILE. Ticks only while the page is visible, so the first tick after
 * a gap is the page being opened: rescan then. */
void FileTick(SettingsSlot& slot, float phys, uint32_t t_ms)
{
    (void)slot;
    const bool reentered = (t_ms - g.last_tick) > 500u;
    g.last_tick = t_ms;

    if (reentered)
    {
        g.flash_armed = false;
        g.dfu_armed   = false;
        if (g.st != St::Comparing && g.st != St::Erasing && g.st != St::Writing
            && g.st != St::Verifying)
            g.st = St::Unscanned;
    }

    switch (g.st)
    {
        case St::Unscanned:
            scan(t_ms);
            break;
        case St::NoCard:
            /* EnsureMounted rate-limits its own retries. */
            if (ready(t_ms)) scan(t_ms);
            break;
        case St::Listed:
        {
            int s = (int)(phys * (float)g.count);
            if (s < 0) s = 0;
            if (s >= g.count) s = g.count - 1;
            g.sel = s;
            break;
        }
        default:
            break;
    }
}

float hour_at(const ArcGeometry& geo, int i)
{
    return fmodf(geo.start_hour + geo.step_hours * (float)i, 12.0f);
}

void FileRender(const SettingsSlot& slot, LedPanel& L, uint8_t pot,
                const ArcGeometry& geo, uint32_t t_ms)
{
    (void)slot; (void)t_ms;
    L.ClearRing(pot);
    const int arc = geo.arc_leds;

    switch (g.st)
    {
        case St::NoCard:
            L.SetRingByHour(pot, hour_at(geo, arc / 2), L.ScaleGlobal(kNoCard));
            break;
        case St::NoDir:
        case St::Empty:
            L.SetRingByHour(pot, hour_at(geo, arc / 2), L.ScaleGlobal(kNoDir));
            break;
        case St::Unscanned:
            break;
        default:
            for (int i = 0; i < g.count; i++)
            {
                int idx = (g.count == 1) ? arc / 2
                        : (int)lroundf((float)i * (float)(arc - 1) / (float)(g.count - 1));
                L.SetRingByHour(pot, hour_at(geo, idx),
                                L.ScaleGlobal(i == g.sel ? kFileOn : kFileOff));
            }
            break;
    }
}

void fill_ring(LedPanel& L, uint8_t pot, const ArcGeometry& geo, float frac,
               const LedPanel::Rgb& c)
{
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    const int n = (int)lroundf(frac * (float)geo.arc_leds);
    for (int i = 0; i < n; i++)
        L.SetRingByHour(pot, hour_at(geo, i), L.ScaleGlobal(c));
}

/* Pot 1: FLASH. Dip below kArmBelow, then sweep past kFireAbove. */
void FlashTick(SettingsSlot& slot, float phys, uint32_t t_ms)
{
    (void)slot;
    switch (g.st)
    {
        case St::Listed:
            if (phys < kArmBelow) g.flash_armed = true;
            if (g.flash_armed && phys > kFireAbove)
            {
                g.flash_armed = false;
                begin_flash(t_ms);
            }
            break;
        case St::Comparing: step_compare(); break;
        case St::Erasing:   step_erase();  break;
        case St::Writing:   step_write();  break;
        case St::Verifying: step_verify(); break;
        case St::Done:
            /* Hand over. Skip the bootloader's DFU grace period: the image is
             * in the slot, boot it. Does not return. */
            daisy::System::ResetToBootloader(
                daisy::System::BootloaderMode::DAISY_SKIP_TIMEOUT);
            break;
        case St::Same:
        case St::Error:
            /* Back off the pot to acknowledge, then the list comes back. */
            if (phys < kArmBelow) g.st = St::Unscanned;
            break;
        default:
            break;
    }
}

void FlashRender(const SettingsSlot& slot, LedPanel& L, uint8_t pot,
                 const ArcGeometry& geo, uint32_t t_ms)
{
    L.ClearRing(pot);
    switch (g.st)
    {
        case St::Listed:
            /* The arming bar: how far the pot has come toward firing. Dim
             * until the dip has been seen, so a parked pot reads as inert. */
            fill_ring(L, pot, geo, slot.pot.stored,
                      g.flash_armed ? kArm : LedPanel::Scale(kArm, 0.25f));
            break;
        case St::Comparing: fill_ring(L, pot, geo, g.progress, kCompare); break;
        case St::Same:      fill_ring(L, pot, geo, 1.0f, kSame);         break;
        case St::Erasing:   fill_ring(L, pot, geo, g.progress, kErase);  break;
        case St::Writing:   fill_ring(L, pot, geo, g.progress, kWrite);  break;
        case St::Verifying: fill_ring(L, pot, geo, g.progress, kVerify); break;
        case St::Done:      fill_ring(L, pot, geo, 1.0f, kDone);         break;
        case St::Error:
        {
            /* Blink so it cannot be mistaken for a colour choice. */
            const bool on = ((t_ms / 250u) & 1u) != 0u;
            if (on) fill_ring(L, pot, geo, 1.0f, kError);
            break;
        }
        default:
            break;
    }
}

/* Pot 2: DFU. Same gesture; reboot into update mode and stay there. */
void DfuTick(SettingsSlot& slot, float phys, uint32_t t_ms)
{
    (void)slot; (void)t_ms;
    if (g.st == St::Comparing || g.st == St::Erasing || g.st == St::Writing
        || g.st == St::Verifying) return;
    if (phys < kArmBelow) g.dfu_armed = true;
    if (g.dfu_armed && phys > kFireAbove)
        daisy::System::ResetToBootloader(
            daisy::System::BootloaderMode::DAISY_INFINITE_TIMEOUT);
}

void DfuRender(const SettingsSlot& slot, LedPanel& L, uint8_t pot,
               const ArcGeometry& geo, uint32_t t_ms)
{
    (void)t_ms;
    L.ClearRing(pot);
    fill_ring(L, pot, geo, slot.pot.stored,
              g.dfu_armed ? kDfu : LedPanel::Scale(kDfu, 0.25f));
}

/* Custom slots get no catch from Settings; track the raw pot so the render
 * can draw the arming bar. */
void TrackPot(SettingsSlot& slot, float phys)
{
    slot.pot.stored = phys;
    slot.pot.caught = true;
}

void FlashTickTracked(SettingsSlot& slot, float phys, uint32_t t_ms)
{
    TrackPot(slot, phys);
    FlashTick(slot, phys, t_ms);
}

void DfuTickTracked(SettingsSlot& slot, float phys, uint32_t t_ms)
{
    TrackPot(slot, phys);
    DfuTick(slot, phys, t_ms);
}

} // namespace

namespace picker {

void Install(Settings& settings, uint8_t page, SdCard& sd, AlchemyLab& hw)
{
    g.sd = &sd;
    g.hw = &hw;

    /* .axi_bss is NOLOAD: clear before first use. */
    memset(&s_fil, 0, sizeof s_fil);
    memset(&s_dir, 0, sizeof s_dir);
    memset(&s_fno, 0, sizeof s_fno);
    memset(s_chunk, 0, sizeof s_chunk);

    settings.Page(page)
        .Name("Firmware")
        .Help("Swap firmware from the SD card. Put `.bin` files built for "
              "the Alchemy Lab bootloader in a folder named `alchemy` on "
              "the card. **File** picks one; **Flash** writes it, verifies "
              "it and reboots into it -- turn the pot down first, then all "
              "the way up. Picking the firmware that is already running "
              "changes nothing and does not reboot. **DFU** reboots into the bootloader's update mode "
              "the same way, for firmware without a picker of its own.");

    settings.Page(page).Pot(0).Custom()
        .Tick(FileTick).Render(FileRender)
        .Ident("fw.file").Name("File")
        .Help("Which file in `/alchemy` to flash. One dot per file, the "
              "chosen one bright. Red: no card. Orange: no `alchemy` folder "
              "or no `.bin` files in it.");

    settings.Page(page).Pot(1).Custom()
        .Tick(FlashTickTracked).Render(FlashRender)
        .Ident("fw.flash").Name("Flash")
        .Help("Turn down past the arming point, then all the way up. Teal "
              "while it checks the file against the running firmware; a solid "
              "green ring means they are the same, so nothing is written and "
              "nothing reboots (turn down to go back to the list). Otherwise "
              "orange while erasing, blue while writing, green while verifying, "
              "white and a reboot when the image checks out. Blinking red: "
              "it failed and nothing was booted; turn down to try again.");

    settings.Page(page).Pot(2).Custom()
        .Tick(DfuTickTracked).Render(DfuRender)
        .Ident("fw.dfu").Name("DFU")
        .Help("Turn down, then all the way up, to reboot into the "
              "bootloader's update mode and stay there for a USB flash.");
}

bool Busy()
{
    return g.st == St::Comparing || g.st == St::Erasing || g.st == St::Writing
        || g.st == St::Verifying;
}

} // namespace picker

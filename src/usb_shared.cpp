/* usb_shared -- see usb_shared.h. */
#include "usb_shared.h"

#include <string.h>

#include "daisy_seed.h"
#include "ff.h"

namespace usbshared
{

namespace
{
constexpr const char* kPath = "/alchemy/usb.cfg";

/* SDMMC DMA reaches these; see alchemy/storage/sd_card.h. */
alignas(32) ALCHEMY_SDMMC_BSS FIL  s_fil;
alignas(32) ALCHEMY_SDMMC_BSS char s_buf[32];
} // namespace

int Load(alchemy::SdCard& sd, uint32_t timeout_ms)
{
    const uint32_t t0 = daisy::System::GetNow();
    while (!sd.EnsureMounted(daisy::System::GetNow()))
    {
        if (daisy::System::GetNow() - t0 > timeout_ms) return -1;
        daisy::System::Delay(5);
    }
    if (f_open(&s_fil, kPath, FA_READ) != FR_OK) return -1;
    UINT n = 0;
    memset(s_buf, 0, sizeof s_buf);
    const FRESULT r = f_read(&s_fil, s_buf, sizeof s_buf - 1, &n);
    f_close(&s_fil);
    if (r != FR_OK || n == 0) return -1;
    if (s_buf[0] == 'l' || s_buf[0] == 'L') return 1;
    if (s_buf[0] == 'm' || s_buf[0] == 'M') return 0;
    return -1;
}

bool Save(alchemy::SdCard& sd, int mode, uint32_t now_ms)
{
    if (!sd.EnsureMounted(now_ms)) return false;
    if (f_open(&s_fil, kPath, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return false;
    const char* text = mode ? "launchpad\n" : "mac\n";
    strncpy(s_buf, text, sizeof s_buf - 1);
    UINT w = 0;
    const FRESULT r = f_write(&s_fil, s_buf, (UINT)strlen(s_buf), &w);
    f_close(&s_fil);
    return r == FR_OK && w == strlen(s_buf);
}

} // namespace usbshared

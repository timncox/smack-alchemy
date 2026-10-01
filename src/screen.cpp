/* screen -- see screen.h. */
#include "screen.h"

#include <string.h>
#include "daisy_seed.h"
#include "hid/disp/oled_display.h"
#include "oled_ssd1305.h"

using namespace daisy;

namespace screen
{
namespace
{
using Display = OledDisplay<oled::SSD1305Driver>;

constexpr uint32_t kTestCardMs = 3000u;
constexpr uint32_t kFrameMs    = 33u; /* <= 30 redraws a second */

Display     display;
bool        ready      = false;
bool        dirty      = true;
bool        card_done  = false;
uint32_t    start_ms   = 0u;
uint32_t    last_ms    = 0u;
const char* card_name  = "";
char        small_[24] = "";
char        big_[16]   = "";

void copy(char* dst, size_t n, const char* src)
{
    size_t i = 0;
    for (; src && src[i] && i + 1 < n; i++) dst[i] = src[i];
    dst[i] = '\0';
}

void DrawTestCard(uint32_t t)
{
    const int w = (int)display.Width(), h = (int)display.Height();
    display.Fill(false);
    display.DrawRect(0, 0, w - 1, h - 1, true, false);
    display.DrawRect(2, 2, 7, 7, true, true); /* top-left marker */
    display.SetCursor(12, 3);
    display.WriteString(card_name, Font_7x10, true);
    display.SetCursor(12, 15);
    display.WriteString("1U OLED TEST", Font_6x8, true);
    const int x = 2 + (int)((t / 20u) % (uint32_t)(w - 14));
    display.DrawRect(x, h - 5, x + 9, h - 3, true, true);
    display.Update();
}

void DrawText()
{
    display.Fill(false);
    display.SetCursor(0, 0);
    display.WriteString(small_, Font_7x10, true);
    display.SetCursor(0, 13);
    display.WriteString(big_, Font_11x18, true);
    display.Update();
}
} // namespace

void Init(const char* name)
{
    card_name = name ? name : "";
    Display::Config cfg; /* SSD1305Driver::Config sets the header pins */
    display.Init(cfg);
    start_ms = System::GetNow();
    ready    = true;
}

void Show(const char* small_line, const char* big_line)
{
    char s[sizeof small_], b[sizeof big_];
    copy(s, sizeof s, small_line);
    copy(b, sizeof b, big_line);
    if (strcmp(s, small_) == 0 && strcmp(b, big_) == 0) return;
    memcpy(small_, s, sizeof s);
    memcpy(big_, b, sizeof b);
    dirty = true;
}

void Poll(uint32_t now_ms)
{
    if (!ready || now_ms - last_ms < kFrameMs) return;
    last_ms = now_ms;
    if (!card_done)
    {
        if (now_ms - start_ms < kTestCardMs)
        {
            DrawTestCard(now_ms - start_ms);
            return;
        }
        card_done = true;
        dirty     = true;
    }
    if (!dirty) return;
    dirty = false;
    DrawText();
}

} // namespace screen

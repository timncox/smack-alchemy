/*
 * screen -- the optional 1U OLED on the rear header (see oled_ssd1305.h for
 * the wiring). Two lines: a small one on top, a big one under it.
 *
 *   screen::Init("SMACK");          after hw.Init(), before StartAudio()
 *   screen::Show("PUNCH FX", "Tape Stop");   any time; cheap if unchanged
 *   screen::Poll(now);              from the control loop, never the audio ISR
 *
 * For the first 3 s after Init it draws a test card instead: a border, a
 * filled block in the TOP-LEFT corner, the name, and a bar sliding along the
 * bottom. Blank = power, reset or wiring; block in the wrong corner = mirrored
 * or flipped; border cut off = column/page offset. Nothing attached is fine:
 * the SPI is transmit-only.
 *
 * Shared by copy across the Alchemy family.
 */
#pragma once

#include <stdint.h>

namespace screen
{
void Init(const char* name);
void Show(const char* small_line, const char* big_line);
void Poll(uint32_t now_ms);
} // namespace screen

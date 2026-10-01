/*
 * SSD1305 driver for a 128x32 OLED on the Alchemy Lab's rear header, shaped
 * like libDaisy's SSD130xDriver so OledDisplay<> (text, fonts, lines) sits on
 * top of it unchanged.
 *
 * The panel is the 2.23" 128x32 EastRising ER-OLEDM023-1 used by the Expert
 * Sleepers / Motley Modular 1U display tiles. libDaisy's SSD130x driver is
 * for the SSD1306/1309 (charge pump, 128-column RAM); the SSD1305 has 132
 * columns and an external VCC, so it needs its own init. The sequence and
 * the page/column offsets of 4 are Adafruit_SSD1305's 128x32 settings,
 * untested on this panel: if the image is shifted, change kColOffset /
 * kPageOffset; if it is mirrored or upside down, flip kSegRemap / kComScan.
 *
 * Transport: libDaisy's SSD130x4WireSpiTransport on SPI1, which already uses
 * the header's SPI pins (SCK PG11 = pin 3, MOSI PB5 = pin 4, NSS PG10 = pin 6,
 * DC PB4 = pin 9). Its default reset (PB15) is the front USB-C's D+, so the
 * config below moves reset to PB6 (pin 8). Transmit-only: with nothing on the
 * header it just clocks into the air, so it is safe to leave on.
 *
 * "Pin" numbers are Hermetic's SDK drawing positions, not IDC odd/even
 * contacts (hermeticmodular.com/docs/hardware#expansion-header). Board-side,
 * marked end at left, Seed below: the top row is 1..10 left to right, so the
 * whole display (GND 2, SCK 3, MOSI 4, CS 6, RESET 8, DC 9) is on the top
 * row. Position 1 (top) is -12 V and 11 (below it) is +12 V. Never power the
 * display from 15 (3V3A); give it its own 3.3 V.
 *
 * Shared by copy across the Alchemy family.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "daisy_core.h"
#include "dev/oled_ssd130x.h"

namespace oled
{

class SSD1305Driver
{
  public:
    static constexpr size_t  kWidth      = 128;
    static constexpr size_t  kHeight     = 32;
    static constexpr uint8_t kColOffset  = 4;    /* 132-column RAM, panel at 4..131 */
    static constexpr uint8_t kPageOffset = 4;    /* rows start at RAM page 4        */
    static constexpr uint8_t kSegRemap   = 0xA1; /* 0xA0 mirrors left-right         */
    static constexpr uint8_t kComScan    = 0xC8; /* 0xC0 flips top-bottom           */

    struct Config
    {
        daisy::SSD130x4WireSpiTransport::Config transport_config;
        Config()
        {
            transport_config.Defaults();
            transport_config.pin_config.reset = daisy::Pin(daisy::PORTB, 6);
            /* SPI1 runs from PLL2; /16 keeps SCLK at a few MHz, well inside
             * what an SSD1305 accepts. 512 bytes still go out in ~1-3 ms. */
            transport_config.spi_config.baud_prescaler
                = daisy::SpiHandle::Config::BaudPrescaler::PS_16;
        }
    };

    void Init(Config config)
    {
        transport_.Init(config.transport_config); /* pulses reset */
        static const uint8_t init[] = {
            0xAE,             /* display off                         */
            0x00 | 0x0,       /* low column 0                        */
            0x10 | 0x0,       /* high column 0                       */
            0x40,             /* start line 0                        */
            0x2E,             /* scrolling off                       */
            0x81, 0x32,       /* contrast                            */
            0x82, 0x10,       /* brightness (area colour bank)       */
            kSegRemap,        /* segment remap                       */
            0xA6,             /* normal, not inverted                */
            0xA8, 0x3F,       /* multiplex 1/64 (Adafruit's 128x32)  */
            0xAD, 0x8E,       /* master config: external VCC         */
            kComScan,         /* COM scan direction                  */
            0xD3, 0x40,       /* display offset                      */
            0xD5, 0xF0,       /* clock divide / oscillator           */
            0xD8, 0x05,       /* area colour off, low power          */
            0xD9, 0xF1,       /* pre-charge                          */
            0xDA, 0x12,       /* COM pins                            */
            0x91, 0x3F, 0x3F, 0x3F, 0x3F, /* look-up table           */
        };
        for (size_t i = 0; i < sizeof(init); i++)
            transport_.SendCommand(init[i]);
        Fill(false);
        Update();
        transport_.SendCommand(0xAF); /* display on */
    }

    size_t Width() const { return kWidth; }
    size_t Height() const { return kHeight; }

    void DrawPixel(uint_fast8_t x, uint_fast8_t y, bool on)
    {
        if (x >= kWidth || y >= kHeight)
            return;
        uint8_t& b = buffer_[x + (y / 8) * kWidth];
        const uint8_t bit = (uint8_t)(1u << (y % 8));
        b = on ? (uint8_t)(b | bit) : (uint8_t)(b & ~bit);
    }

    void Fill(bool on)
    {
        for (size_t i = 0; i < sizeof(buffer_); i++)
            buffer_[i] = on ? 0xFF : 0x00;
    }

    void SetContrast(uint8_t c)
    {
        transport_.SendCommand(0x81);
        transport_.SendCommand(c);
    }

    /* Blocking: 4 pages x 128 bytes on SPI. Call from the control loop,
     * never from the audio callback. */
    void Update()
    {
        for (uint8_t p = 0; p < kHeight / 8; p++)
        {
            uint8_t cmd[3] = {(uint8_t)(0xB0 + p + kPageOffset),
                              (uint8_t)(0x10 + (kColOffset >> 4)),
                              (uint8_t)(kColOffset & 0x0F)};
            transport_.SendCommands(cmd, sizeof(cmd));
            transport_.SendData(&buffer_[kWidth * p], kWidth);
        }
    }

    bool UpdateFinished() { return true; }

  private:
    daisy::SSD130x4WireSpiTransport transport_;
    uint8_t                         buffer_[kWidth * kHeight / 8];
};

} // namespace oled

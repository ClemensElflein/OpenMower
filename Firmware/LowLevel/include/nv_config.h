/*
 * Copyright 2024 Jörg Ebeling (Apehaenger)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.* Testing/examples for reading/writing flash on the RP2040.
 *
 * Heavily copied from as well as must-read:
 * https://github.com/MakerMatrix/RP2040_flash_programming
 */

#include <Arduino.h>

// #define NV_CONFIG_MAX_SAVE_INTERVAL 300000 // Don't save more often than every 5 minutes
#define NV_CONFIG_MAX_SAVE_INTERVAL 10000 // DBG: Don't save more often than every 10 seconds

#define LL_CONFIG_BIT_HL_CONFIG_RECEIVED 0x1
#define LL_CONFIG_BIT_DFPIS5V 0x2

namespace nv_config
{
#pragma pack(push, 1)
    // Config struct with reasonable defaults
    struct Config
    {
        // Config bitmask:
        // Bit 0: ROS config packet received. See LL_CONFIG_BIT_HL_CONFIG_RECEIVED
        // Bit 1: DFP is 5V (enable full sound). See LL_CONFIG_BIT_DFPIS5V
        uint8_t config_bitmask;
        uint8_t language = 0;          // Sound language index 0 = Folder "01" = English(US), 1 = Folder "49" = German
        uint8_t volume = 100;          // Sound loudness from 0 to 100 %
        uint8_t reserved;              // padding
        uint32_t rain_threshold = 700; // If (stock CoverUI) rain value < rain_threshold then it rains. Expected to differ between C500, SA and SC types
        /* FIXME: Can't see a benefit why a CRC16 within our struct, for the initial config set will ensure anything ;-)
        uint16_t nv_crc;               // CRC 16 used to validate what?*/

        /* Possible future config settings
        uint16_t free;                 // Future config setting
        uint16_t free_n;               // Future config setting
        uint16_t crc_n;                // Future config CRC16 for detection if loaded (old) config already has the new config member */
    } __attribute__((packed));
#pragma pack(pop)

    Config *get(); // Return a pointer to nv_config::config which holds the last saved config, or the first default one. Config member are writable, see save()
    void save();   // Handle a possibly changed nv_config::config member and save it to flash, but only within a defined timeout to ensure wear level protection
}
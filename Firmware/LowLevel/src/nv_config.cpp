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
#include "nv_config.h"
#include "debug.h"
#include <FastCRC.h>

extern "C"
{
#include <hardware/sync.h>
#include <hardware/flash.h>
};

// Set the target offset to the last sector of flash
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

#ifdef DEBUG_PREFIX
#undef DEBUG_PREFIX
#define DEBUG_PREFIX "[NVC] "
#endif

namespace nv_config
{
    static_assert(sizeof(Config) <= FLASH_PAGE_SIZE, "Config struct size larger than page size"); // Probably doesn't harm, but didn't tested

    int first_empty_page = -1;

    Config config; // Last or (if there hasn't been a config flashed before) default config
    uint16_t crc;  // CRC of config

    FastCRC16 CRC16;

    unsigned long next_save_millis = millis() + NV_CONFIG_MAX_SAVE_INTERVAL;

    Config *get()
    {
        DEBUG_PRINTF("FLASH_PAGE_SIZE: %d, FLASH_SECTOR_SIZE: %d, FLASH_BLOCK_SIZE: %d, PICO_FLASH_SIZE_BYTES: %d, XIP_BASE: 0x%x, FLASH_TARGET_OFFSET: %d\n",
                     FLASH_PAGE_SIZE, FLASH_SECTOR_SIZE, FLASH_BLOCK_SIZE, PICO_FLASH_SIZE_BYTES, XIP_BASE,
                     FLASH_TARGET_OFFSET);

        // Read flash until first empty page found
        unsigned int page; // prevent comparison of unsigned and signed int
        for (page = 0; page < FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE; page++)
        {
            int addr = XIP_BASE + FLASH_TARGET_OFFSET + (page * FLASH_PAGE_SIZE);
            Config *ptr = (Config *)addr;

            // Instead testing all of our first config member, lets cast the struct to a long long int, which is save to identify an empty page
            long long trix = *((long long *)ptr);
            DEBUG_PRINTF("Config in page %d (at 0x%x) = config_bitmask: 0b" PRINTF_BINARY_PATTERN_INT8 ", rain_threshold: %d, language: %d, volume: %d, trix: %lld\n ",
                         page, int(ptr), PRINTF_BYTE_TO_BINARY_INT8(ptr->config_bitmask), ptr->rain_threshold, ptr->language, ptr->volume, trix);
            if (trix == -1)
            {
                first_empty_page = page;
                DEBUG_PRINTF("First empty page is %d\n", first_empty_page);
                break;
            }
        }

        // Copy flash config if found
        if (page > 0)
        {
            // Deep copy flash-config to our local one via implicit copy constructor
            config = *(Config *)(XIP_BASE + FLASH_TARGET_OFFSET + ((page - 1) * FLASH_PAGE_SIZE));
        }
        DEBUG_PRINTF("Default or Last config's volume: %d\n", config.volume);

        // CRC for simple checking if config has changed
        crc = CRC16.ccitt((uint8_t *)&config, sizeof(Config));
        DEBUG_PRINTF("Config CRC: 0x%x\n", crc);

        return &config;
    }

    void save()
    {
        // Wear level protection
        if (millis() < next_save_millis)
            return;
        next_save_millis = millis() + NV_CONFIG_MAX_SAVE_INTERVAL;

        // Check CRC if config changed
        uint16_t check_crc = CRC16.ccitt((uint8_t *)&config, sizeof(Config));
        DEBUG_PRINTF("Actual- vs. stored- config CRC: 0x%x ?= 0x%x\n", check_crc, crc);
        if (check_crc == crc)
            return; // CRC doesn't changed

        // If no empty page got found during get(), let's erase the flash
        if (first_empty_page < 0)
        {
            DEBUG_PRINTF("Full sector, erase...\n");
            uint32_t ints = save_and_disable_interrupts();
            flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
            first_empty_page = 0;
            restore_interrupts(ints);
        }

        DEBUG_PRINTF("Write to page %d, volume %d...\n", first_empty_page, config.volume);
        uint32_t ints = save_and_disable_interrupts();
        flash_range_program(FLASH_TARGET_OFFSET + (first_empty_page * FLASH_PAGE_SIZE), (uint8_t *)&config, FLASH_PAGE_SIZE);
        restore_interrupts(ints);
        crc = check_crc;
        first_empty_page++;
        if (first_empty_page >= FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE)
            first_empty_page = -1; // Last page written, indicated sector full
    }
}
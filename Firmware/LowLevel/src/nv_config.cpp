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
 * Heavily inspired, copied from, as well as must-read:
 * https://github.com/MakerMatrix/RP2040_flash_programming
 */
#include "nv_config.h"

#include <FastCRC.h>

#include <memory>

#include "debug.h"

extern "C" {
#include <hardware/flash.h>
#include <hardware/sync.h>
};

#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)  // Target offset to the last sector of flash
#define NV_MIN_RECORD_SIZE (sizeof(Record) - sizeof(Config))             // Minimum Record size (Record size without Config member)
#define NV_PAGES_IN_SECTOR (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE)         // How much pages fit into one sector
#define NV_RECORD_CRC_DATALEN (sizeof(Record) - sizeof(Config) - sizeof(cur_record.crc))

#ifdef DEBUG_PREFIX
#undef DEBUG_PREFIX
#define DEBUG_PREFIX "[NVC] "
#endif

namespace nv_config {
static_assert(sizeof(Record) <= FLASH_PAGE_SIZE, "Record struct size larger than page size");

int first_empty_page = -1;  // First empty page in sector
int last_used_page = -1;    // Last used page with non-empty data
int last_record_pos = -1;   // Last record position within page (page_buf)

uint8_t page_buf[FLASH_PAGE_SIZE];  // Page buffer required for subsequential flash_range_program() of the same page
Record cur_record;                  // Current (last or (if there hasn't been a Record flashed before) default) Record
uint16_t config_crc;                // CRC of current Record->config used for config change detection

FastCRC16 CRC16;

unsigned long next_save_millis = millis() + NV_CONFIG_MAX_SAVE_INTERVAL;

Config *get() {
    DEBUG_PRINTF("FLASH_PAGE_SIZE: %d, FLASH_SECTOR_SIZE: %d, FLASH_BLOCK_SIZE: %d, PICO_FLASH_SIZE_BYTES: %d, XIP_BASE: 0x%x, FLASH_TARGET_OFFSET: %d\n",
                 FLASH_PAGE_SIZE, FLASH_SECTOR_SIZE, FLASH_BLOCK_SIZE, PICO_FLASH_SIZE_BYTES, XIP_BASE, FLASH_TARGET_OFFSET);
    DEBUG_PRINTF("NV_PAGES_IN_SECTOR: %d, sizeof(Record): %d, sizeof(Config): %d, possible num of records within one page: %d\n",
                 NV_PAGES_IN_SECTOR, sizeof(Record), sizeof(Config), FLASH_PAGE_SIZE / sizeof(Record));

    // Read flash until first empty page found
    int addr;
    unsigned int page;
    int32_t *p;
    for (page = 0; page < FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE; page++) {
        p = (int32_t *)(XIP_BASE + FLASH_TARGET_OFFSET + (page * FLASH_PAGE_SIZE));

        // 0xFFFFFFFF cast as an int is -1 so this is how we detect an empty page
        DEBUG_PRINTF("Page %d (0x%x): record_id = %ld (0x%lx)\n", page, int(p), *p, *p);
        if (*p == -1 && first_empty_page < 0) {
            first_empty_page = page;
            break;
        }
        last_used_page = page;
    }
    DEBUG_PRINTF("last_used_page %d, first_empty_page %d\n", last_used_page, first_empty_page);

    // Find last written Record
    if (last_used_page >= 0) {
        // RLoop over last_used_page to find latest record via it's record.id
        DEBUG_PRINTF("Last possible record (unaligned) = FLASH_PAGE_SIZE %d - NV_MIN_RECORD_SIZE %d = %u\n", FLASH_PAGE_SIZE, NV_MIN_RECORD_SIZE, FLASH_PAGE_SIZE - NV_MIN_RECORD_SIZE);
        int addr = XIP_BASE + FLASH_TARGET_OFFSET + (last_used_page * FLASH_PAGE_SIZE);
        // Calc last possible record offset without config, for the case that Record.config get changed, and pointer-align to "next multiple of"
        int last_possible_record_offset = (FLASH_PAGE_SIZE - NV_MIN_RECORD_SIZE) - ((FLASH_PAGE_SIZE - NV_MIN_RECORD_SIZE) % NV_RECORD_ALIGNMENT) + NV_RECORD_ALIGNMENT;
        DEBUG_PRINTF("last_possible_record_offset (aligned) 0x%x for Record.id alignment of %d\n", last_possible_record_offset, NV_RECORD_ALIGNMENT);
        // Record test_record;
        for (size_t i = last_possible_record_offset; i >= 0; i = i - NV_RECORD_ALIGNMENT) {
            uint32_t *p = (uint32_t *)(addr + i);
            if (*p != NV_RECORD_ID)
                continue;

            // Validate if the found NV_RECORD_ID is a valid Record.id
            DEBUG_PRINTF("Page %d, offset 0x%x (at %p): Record.id prospect = 0x%lx\n", last_used_page, i, p, *p);
            Record test_record;
            memcpy(&test_record, (uint8_t *)p, sizeof(Record));
            DEBUG_PRINTF("Test- Record: num_sector_erase: %ld, num_page_write: %ld, crc: 0x%lx\n", test_record.num_sector_erase, test_record.num_page_write, test_record.crc);
            uint16_t test_crc = CRC16.ccitt((uint8_t *)&test_record, NV_RECORD_CRC_DATALEN);
            DEBUG_PRINTF("Test- Record CRC 0x%x of datalen %d\n", test_crc, NV_RECORD_CRC_DATALEN);
            if (test_crc != test_record.crc)
                continue;

            DEBUG_PRINTF("Test- Record is valid. Buffer the full flash page\n");
            memcpy(&page_buf, (uint8_t *)addr, FLASH_PAGE_SIZE);
            memcpy(&cur_record, &test_record, sizeof(Record));
            last_record_pos = i;
            break;
        }
    }
    DEBUG_PRINTF("Last (or default) config's volume: %d\n", cur_record.config.volume);

    // CRC for simple checking if config has changed
    config_crc = CRC16.ccitt((uint8_t *)&cur_record.config, sizeof(Config));
    DEBUG_PRINTF("cur_record.config CRC: 0x%x\n", config_crc);

    return &cur_record.config;
}

void delayedSaveChanges() {
    // Wear level protection
    if (millis() < next_save_millis)
        return;
    next_save_millis = millis() + NV_CONFIG_MAX_SAVE_INTERVAL;

    // Check CRC if config changed
    uint16_t check_crc = CRC16.ccitt((uint8_t *)&cur_record.config, sizeof(Config));
    DEBUG_PRINTF("Actual- vs. stored- config CRC: 0x%x ?= 0x%x\n", check_crc, config_crc);
    if (check_crc == config_crc)
        return;  // Record.config doesn't changed

    // Prepare record position within page buffer
    if (last_record_pos == -1)
        last_record_pos = 0;
    else {
        last_record_pos += sizeof(Record);
        last_record_pos = last_record_pos - (last_record_pos % NV_RECORD_ALIGNMENT) + NV_RECORD_ALIGNMENT;  // Adjust to "next multiple of" NV_RECORD_ALIGNMENT
    }
    if ((last_record_pos + sizeof(Record)) > FLASH_PAGE_SIZE) {
        // Will not fit into the current page anymore
        last_used_page++;
        last_record_pos = 0;
        first_empty_page++;
        std::fill_n(page_buf, FLASH_PAGE_SIZE, 0xff);  // Mark page buffer as (flash) erased
    }

    // If no empty page got found during get(), or last page got eaten, let's erase our flash sector
    if (first_empty_page < 0 || last_used_page >= NV_PAGES_IN_SECTOR) {
        DEBUG_PRINTF("Full sector, erase...\n");
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
        restore_interrupts(ints);
        // Reset positioning vars
        last_used_page = 0;
        first_empty_page = 0;
        cur_record.num_sector_erase++;
        std::fill_n(page_buf, FLASH_PAGE_SIZE, 0xff);  // Mark page buffer as (flash) erased
    }

    // Prepare record
    cur_record.num_page_write++;
    cur_record.crc = CRC16.ccitt((uint8_t *)&cur_record, NV_RECORD_CRC_DATALEN);
    DEBUG_PRINTF("Record CRC 0x%x of datalen %d\n", cur_record.crc, NV_RECORD_CRC_DATALEN);

    // memcopy cur_record to page buffer
    memcpy(&page_buf[last_record_pos], &cur_record, sizeof(Record));

    // Write page to flash
    DEBUG_PRINTF("Write to page %d, with new record pos %d: volume %d...\n", last_used_page, last_record_pos, cur_record.config.volume);
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(FLASH_TARGET_OFFSET + (last_used_page * FLASH_PAGE_SIZE), (uint8_t *)page_buf, sizeof(page_buf));
    restore_interrupts(ints);
    config_crc = check_crc;  // Update last saved config CRC
}
}  // namespace nv_config
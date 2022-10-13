// Created by Elmar Elflein on 3/07/22.
// Copyright (c) 2022 Elmar Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
#ifndef _UI_DATATYPES_H_
#define _UI_DATATYPES_H_

#include <stdint.h>
#include <cmath>
// Protocol Header Info
enum TYPE
{
    Get_Version = 0xB0,
    Set_Buzzer = 0xB1,
    Set_LEDs = 0xB2,
    Get_Button = 0xB3
};


// Function definiton for the 18 LEDS
enum LED_id
{
    LED_CHARGING = 0,
    LED_BATTERY_LOW = 1,
    LED_POOR_GPS = 2,
    LED_MOWER_LIFTED = 3,
    LED5 = 4,
    LED6 = 5,
    LED7 = 6,
    LED8 = 7,
    LED9 = 8,
    LED10 = 9,
    LED11 = 10,
    LED_LOCK = 11,
    LED_S2 = 12,
    LED_S1 = 13,
    LED15 = 14,
    LED16 = 15,
    LED17 = 16,
    LED18 = 17
};

enum LED_state {
    LED_off = 0b000,
    LED_blink_slow = 0b101,
    LED_blink_fast = 0b110,
    LED_on = 0b111
};

#pragma pack(push, 1)
struct msg_get_version
{
    uint8_t type; // command type
    uint8_t reserved; // padding
    uint16_t version;
    uint16_t crc; // CRC 16
} __attribute__((packed));
#pragma pack(pop)


#pragma pack(push, 1)
struct msg_set_buzzer
{
    uint8_t type; // command type
    uint8_t repeat; // Repeat X times
    uint8_t on_time; // Signal on time
    uint8_t off_time; // Signal off time
    uint16_t crc; // CRC 16
} __attribute__((packed));
#pragma pack(pop)


/**
 * @brief Use this to update the LED matrix
 * Each LED gets three bits with the following meaning:
 * 0b000 = Off
 * 0b001 = reserved for future use
 * 0b010 = reserved for future use
 * 0b011 = reserved for future use
 * 0b100 = reserved for future use
 * 0b101 = On slow blink
 * 0b110 = On fast blink
 * 0b111 = On
 */
#pragma pack(push, 1)
struct msg_set_leds
{
    uint8_t type; // command type
    uint8_t reserved; // padding
    uint64_t leds;
    uint16_t crc; // CRC 16
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct msg_event_button
{
    uint8_t type; // command type
    uint16_t button_id;
    uint8_t press_duration;
    uint16_t crc; // CRC 16
} __attribute__((packed));
#pragma pack(pop)


void setLed(struct msg_set_leds &msg, int led, uint8_t state);

void setBars7(struct msg_set_leds &msg, double value);

void setBars4(struct msg_set_leds &msg, double value);


#endif // _BttnCtl_HEADER_FILE_

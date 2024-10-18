// Created by Clemens Elflein on 3/07/22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
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

#ifndef _DATATYPES_H
#define _DATATYPES_H

#include <stdint.h>

#define PACKET_ID_LL_STATUS 1
#define PACKET_ID_LL_IMU 2
#define PACKET_ID_LL_UI_EVENT 3
#define PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ 0x21 // ll_high_level_config and request config from receiver
#define PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP 0x22 // ll_high_level_config response
#define PACKET_ID_LL_HEARTBEAT 0x42
#define PACKET_ID_LL_HIGH_LEVEL_STATE 0x43

enum HighLevelMode {
    MODE_IDLE = 1, // ROS connected, idle mode
    MODE_AUTONOMOUS = 2, // ROS connected, Autonomous mode, either mowing or docking or undocking
    MODE_RECORDING = 3 // ROS connected, Manual mode during recording etc
};

#define LL_EMERGENCY_BIT_LATCH 0b00000001
#define LL_EMERGENCY_BIT_HALL1 0b00001000 // Lift1
#define LL_EMERGENCY_BIT_HALL2 0b00010000 // Lift2
#define LL_EMERGENCY_BIT_HALL3 0b00000010 // Stop1
#define LL_EMERGENCY_BIT_HALL4 0b00000100 // Stop2

#define LL_EMERGENCY_BIT_LIFT1 LL_EMERGENCY_BIT_HALL1
#define LL_EMERGENCY_BIT_LIFT2 LL_EMERGENCY_BIT_HALL2
#define LL_EMERGENCY_BITS_LIFT (LL_EMERGENCY_BIT_LIFT1 | LL_EMERGENCY_BIT_LIFT2)
#define LL_EMERGENCY_BIT_STOP1 LL_EMERGENCY_BIT_HALL3
#define LL_EMERGENCY_BIT_STOP2 LL_EMERGENCY_BIT_HALL4
#define LL_EMERGENCY_BITS_STOP (LL_EMERGENCY_BIT_STOP1 | LL_EMERGENCY_BIT_STOP2)

#define LIFT1_IS_INVERTED 0
#define LIFT2_IS_INVERTED 0

#define LL_STATUS_BIT_UI_AVAIL 0b10000000

#pragma pack(push, 1)
struct ll_status {
    // Type of this message. Has to be PACKET_ID_LL_STATUS.
    uint8_t type;
    // Bitmask for rain, sound, powers etc
    // Bit 0: Initialized (i.e. setup() was a success). If this is 0, all other bits are meaningless.
    // Bit 1: Raspberry Power
    // Bit 2: Charging enabled
    // Bit 3: don't care (reserved for ESC shutdown PR)
    // Bit 4: Rain detected
    // Bit 5: Sound available
    // Bit 6: Sound busy
    // Bit 7: UI Board available
    uint8_t status_bitmask;
    // USS range in m
    float uss_ranges_m[5];
    // Emergency bitmask:
    // Bit 0: Emergency latch
    // Bit 1: Emergency/Hall 3 (Stop1) active
    // Bit 2: Emergency/Hall 4 (Stop2) active
    // Bit 3: Emergency/Hall 1 (Lift1) active
    // Bit 4: Emergency/Hall 2 (Lift2) active
    // Bit 5: Not an emergency but probably usable for pause via SA Handle?
    uint8_t emergency_bitmask;
    // Charge voltage
    float v_charge;
    // System voltage
    float v_battery;
    // Charge current
    float charging_current;
    uint8_t batt_percentage;
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct ll_imu {
    // Type of this message. Has to be PACKET_ID_LL_IMU.
    uint8_t type;
    // Time since last message in milliseconds.
    uint16_t dt_millis;
    // Acceleration[m^s2], Gyro[rad/s] and magnetic field[uT]
    float acceleration_mss[3];
    float gyro_rads[3];
    float mag_uT[3];
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct ll_heartbeat {
    // Type of this message. Has to be PACKET_ID_LL_HEARTBEAT.
    uint8_t type;
    // True, if emergency should be engaged (e.g. navigation error, ...)
    // False to not change the latch
    uint8_t emergency_requested;
    // True, if emergency condition should be reset
    // False to not change the latch
    uint8_t emergency_release_requested;
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)


#pragma pack(push, 1)
struct ll_high_level_state {
    // Type of this message. Has to be PACKET_ID_LL_HIGH_LEVEL_STATE
    uint8_t type;
    uint8_t current_mode; // see HighLevelMode
    uint8_t gps_quality;   // GPS quality in percent (0-100)
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct ll_ui_event {
    // Type of this message. Has to be PACKET_ID_LL_UI_EVENT
    uint8_t type;
    uint8_t button_id; 
    uint8_t press_duration;   // 0 for single press, 1 for long, 2 for very long press
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#define LL_HIGH_LEVEL_CONFIG_BIT_DFPIS5V (1 << 0)                  // Enable full sound via mower_config env var "OM_DFP_IS_5V"
#define LL_HIGH_LEVEL_CONFIG_BIT_BACKGROUND_SOUNDS (1 << 1)        // Enable background sounds
#define LL_HIGH_LEVEL_CONFIG_BIT_IGNORE_CHARGING_CURRENT (1 << 2)  // Ignore charging current

#define LL_HIGH_LEVEL_CONFIG_BIT_HL_IS_LEADING (LL_HIGH_LEVEL_CONFIG_BIT_DFPIS5V | LL_HIGH_LEVEL_CONFIG_BIT_BACKGROUND_SOUNDS | LL_HIGH_LEVEL_CONFIG_BIT_IGNORE_CHARGING_CURRENT)

typedef char iso639_1[2]; // Two char ISO 639-1 language code

enum class HallMode : unsigned int {
    OFF = 0,
    LIFT_TILT,  // Wheel lifted and/or wheels tilted functionality
    STOP,       // Stop mower
    PAUSE,      // Pause the mower (not yet implemented in ROS)
    UNDEFINED   // This is used by foreign side to inform that it doesn't has a configuration for this sensor
};

// FIXME: Decide later which is more comfortable, activeLow = 0 | 1
enum class HallLevel : unsigned int {
    ACTIVE_LOW = 0,  // If Hall-Sensor (or button) is active/triggered we've this level on our GPIO
    ACTIVE_HIGH
};

#pragma pack(push, 1)
struct HallConfig {
    HallMode mode : 3;
    HallLevel level : 1;
} __attribute__((packed));
#pragma pack(pop)

#define MAX_HALL_INPUTS 10  // How much Hall-inputs we support. 4 * OM + 6 * Stock-CoverUI + 0 spare (because not yet required to make it fixed)

// LL/HL config packet, bi-directional, flexible-length, with defaults for YF-C500.
#pragma pack(push, 1)
struct ll_high_level_config {
    // ATTENTION: This is a flexible length struct. It is allowed to grow independently to HL without loosing compatibility,
    //    but never change or restructure already published member, except you really know their consequences.

    // uint8_t type; Just for illustration. Get set in wire buffer with type PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ or PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP

    uint8_t config_bitmask = 0;            // See LL_HIGH_LEVEL_CONFIG_BIT_*
    uint16_t rain_threshold = 700;         // If (stock CoverUI) rain value < rain_threshold then it rains. Expected to differ between C500, SA and SC types (0xFFFF = unknown)
    float v_charge_cutoff = 30.0f;         // Protective max. charging voltage before charging get switched off (-1 = unknown)
    float i_charge_cutoff = 1.5f;          // Protective max. charging current before charging get switched off (-1 = unknown)
    float v_battery_cutoff = 29.0f;        // Protective max. battery voltage before charging get switched off (-1 = unknown)
    float v_battery_empty = 21.7f + 0.3f;  // Empty battery voltage used for % calc of capacity (-1 = unknown)
    float v_battery_full = 28.7f - 0.3f;   // Full battery voltage used for % calc of capacity (-1 = unknown)
    uint16_t lift_period = 100;            // Period (ms) for both wheels to be lifted in order to count as emergency (0 = disable, 0xFFFF = unknown). This is to filter uneven ground
    uint16_t tilt_period = 2500;           // Period (ms) for a single wheel to be lifted in order to count as emergency (0 = disable, 0xFFFF = unknown). This is to filter uneven ground
    float shutdown_esc_max_pitch = 0.0f;   // Do not shutdown ESCs if absolute pitch angle is greater than this (0 = disable, 0xffff = unknown) (to be implemented)
    iso639_1 language = {'e', 'n'};        // ISO 639-1 (2-char) language code (en, de, ...)
    uint8_t volume = 80;                   // Volume (0-100%) feedback (if directly changed i.e. via CoverUI or WebApp) (0xff = do not change)
    HallConfig hall_configs[MAX_HALL_INPUTS] = {
        {HallMode::LIFT_TILT, HallLevel::ACTIVE_LOW},  // [0] OM Hall-1 input (Lift1)
        {HallMode::LIFT_TILT, HallLevel::ACTIVE_LOW},  // [1] OM Hall-2 input (Lift2)
        {HallMode::STOP, HallLevel::ACTIVE_LOW},       // [2] OM Hall-3 input (Stop1)
        {HallMode::STOP, HallLevel::ACTIVE_LOW},       // [3] OM Hall-4 input (Stop2)
        // [4] Stock-CoverUI-1 ... [9] Stock-CoverUI-6 default to off
    };
    // INFO: Before adding a new member here: Decide if and how much hall_configs spares do we like to have

    // uint16_t crc;  Just for illustration, that it get appended, but only within the wire buffer
} __attribute__((packed));
#pragma pack(pop)

#endif

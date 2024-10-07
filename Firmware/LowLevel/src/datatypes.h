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

#define LL_HIGH_LEVEL_CONFIG_MAX_COMMS_VERSION 2           // Max. comms packet version supported by this open_mower LL FW
#define LL_HIGH_LEVEL_CONFIG_BIT_DFPIS5V 1 << 0            // Enable full sound via mower_config env var "OM_DFP_IS_5V"
#define LL_HIGH_LEVEL_CONFIG_BIT_EMERGENCY_INVERSE 1 << 1  // Sample, for possible future usage, i.e. for SA-Type emergency

typedef char iso639_1[2]; // Two char ISO 639-1 language code

// FIXME: Better name than HallMode?
enum class HallMode {
    off = 0,
    lift,
    tilt,
    stop,
    pause
};

// Bi-directional LL/HL config packet
#pragma pack(push, 1)
struct ll_high_level_config {
    uint8_t type;
    uint8_t comms_version = LL_HIGH_LEVEL_CONFIG_MAX_COMMS_VERSION;  // Increasing comms packet-version number for packet compatibility (n > 0)
    uint8_t config_bitmask = 0;                                      // See LL_HIGH_LEVEL_CONFIG_BIT_*
    int8_t volume;                                                   // Volume (0-100%) feedback (if directly changed via CoverUI)
    iso639_1 language;                                               // ISO 639-1 (2-char) language code (en, de, ...)
    uint16_t spare1 = 0;                                             // Spare for future use
    uint16_t spare2 = 0;                                             // Spare for future use
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

// Uni-directional HL->LL config packet extension v2, active since ll_high_level_config.comms_version >= 2
// (this might change to an Bi-directional packet if HL needs feedback for one of these fields)
#pragma pack(push, 1)
struct ll_high_level_config_ext_v2 {
    float v_charge_max;       // Max. charging voltage before charging get switched off
    float i_charge_max;       // Max. charging current before charging get switched off
    float v_battery_max;      // Max. battery voltage before charging get switched off
    uint32_t halls_config;    // HallMode per [0:2] OM-Hall-1 ... [9:11] OM-Hall-4, [12:14] Stock-CoverUI-1 ... [27:29] Stock-CoverUI-6 (0xFFFFFF do not change)
    uint16_t halls_inverted;  // 0 = Active high, 1 = Active low (inverted). [0] OM-Hall-1 ... [9] Stock-CoverUI-6. ATTENTION: Opposite than current logic (0xFFFF do not change)
    uint16_t lift_period;     // Period (ms) for both wheels to be lifted in order to count as emergency (0 disable, 0xFFFF do not change). This is to filter uneven ground
    uint16_t tilt_period;     // Period (ms) for a single wheel to be lifted in order to count as emergency (0 disable, 0xFFFF do not change). This is to filter uneven ground
    uint32_t spare1;          // Spare for future use
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#endif

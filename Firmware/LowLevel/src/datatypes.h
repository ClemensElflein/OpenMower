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
#include <functional>

#define PACKET_ID_LL_STATUS 1
#define PACKET_ID_LL_IMU 2
#define PACKET_ID_LL_UI_EVENT 3
#define PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ 0x11 // ll_high_level_config and request config from receiver
#define PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP 0x12 // ll_high_level_config response
#define PACKET_ID_LL_HEARTBEAT 0x42
#define PACKET_ID_LL_HIGH_LEVEL_STATE 0x43

/**
 * @brief Simple class containing only the static methods,
 * for more comfortable as well as more speaking HighLevel-Mode and SubMode handling
 */
#define HL_MODE_MASK 0b11111
#define HL_SUBMODE_SHIFT 6
#define HL_SUBMODE_MASK 0b11
class HighLevelState {
   public:
    enum class Mode : uint8_t {
        IDLE = 1,        // ROS connected, idle mode
        AUTONOMOUS = 2,  // ROS connected, Autonomous mode, either mowing or docking or undocking
        RECORDING = 3,   // ROS connected, Manual mode during recording etc
    };
    enum SubModeIdle : uint8_t {
        AUTONOMOUS_MOWING = 0,  // ?!
    };
    enum SubModeAutonomous : uint8_t {
        MOWING = 0,
        DOCKING = 1,
        UNDOCKING = 2,
    };
    enum SubModeRecording : uint8_t {
        OUTLINE = 1,
        OBSTACLE = 2,
    };

    static Mode getMode(uint8_t t_mode) {
        return Mode(t_mode & HL_MODE_MASK);
    }

    static uint8_t getSubMode(uint8_t t_mode) {
        return (t_mode >> HL_SUBMODE_SHIFT) & HL_SUBMODE_MASK;
    }

    static SubModeAutonomous getAutonomousSubMode(uint8_t t_mode) {
        return SubModeAutonomous(getSubMode(t_mode));
    }
};

// clang-format off
#define LL_EMERGENCY_BIT_LATCH (1 << 0)  // Any emergency latch
#define LL_EMERGENCY_BIT_STOP  (1 << 1)  // Stop
#define LL_EMERGENCY_BIT_LIFT  (1 << 2)  // Lift (or tilt)

// CoverUI will stay with the old emergency_bitmask definition
#define LL_EMERGENCY_BIT_CU_LATCH (1 << 0)  // Any emergency latch
#define LL_EMERGENCY_BIT_CU_STOP1 (1 << 1)  // Stop1
#define LL_EMERGENCY_BIT_CU_STOP2 (1 << 2)  // Stop2
#define LL_EMERGENCY_BIT_CU_LIFT  (1 << 3)  // LIFT | LIFTX (up to CoverUI FW 2.0x)
#define LL_EMERGENCY_BIT_CU_BUMP  (1 << 4)  // LBUMP | RBUMP (up to CoverUI FW 2.0x)
#define LL_EMERGENCY_BIT_CU_LIFTX (1 << 5)  // CoverUI-LIFTX (as of CoverUI FW 2.1x)
#define LL_EMERGENCY_BIT_CU_RBUMP (1 << 6)  // CoverUI-RBUMP (as of CoverUI FW 2.1x)

#define LL_STATUS_BIT_INITIALIZED (1 << 0)
#define LL_STATUS_BIT_RASPI_POWER (1 << 1)
#define LL_STATUS_BIT_CHARGING    (1 << 2)
#define LL_STATUS_BIT_FREE        (1 << 3)
#define LL_STATUS_BIT_RAIN        (1 << 4)
#define LL_STATUS_BIT_SOUND_AVAIL (1 << 5)
#define LL_STATUS_BIT_SOUND_BUSY  (1 << 6)
#define LL_STATUS_BIT_UI_AVAIL    (1 << 7)
// clang-format on

#pragma pack(push, 1)
struct ll_status {
    // Type of this message. Has to be PACKET_ID_LL_STATUS.
    uint8_t type;
    // Bitmask for rain, sound, powers etc
    // Bit 0: Initialized (i.e. setup() was a success). If this is 0, all other bits are meaningless.
    // Bit 1: Raspberry Power
    // Bit 2: Charging enabled
    // Bit 3: ESC power
    // Bit 4: Rain detected
    // Bit 5: Sound available
    // Bit 6: Sound busy
    // Bit 7: UI Board available
    uint8_t status_bitmask;
    // USS range in m
    float uss_ranges_m[5];
    // Emergency bitmask:
    // Bit 0: Emergency latch
    // Bit 1: Emergency/Lift (or tilt)
    // Bit 2: Emergency/Stop
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

enum class OptionState : unsigned int {
    OFF = 0,
    ON,
    UNDEFINED
};

#pragma pack(push, 1)
struct ConfigOptions {
    OptionState dfp_is_5v : 2;
    OptionState background_sounds : 2;
    OptionState ignore_charging_current : 2;
    // Need to block/waster the bits now, to be prepared for future enhancements
    OptionState reserved_for_future_use1 : 2;
    OptionState reserved_for_future_use2 : 2;
    OptionState reserved_for_future_use3 : 2;
    OptionState reserved_for_future_use4 : 2;
    OptionState reserved_for_future_use5 : 2;
} __attribute__((packed));
#pragma pack(pop)
static_assert(sizeof(ConfigOptions) == 2, "Changing size of ConfigOption != 2 will break packet compatibilty");

typedef char iso639_1[2]; // Two char ISO 639-1 language code

enum class HallMode : unsigned int {
    OFF = 0,
    LIFT_TILT,  // Wheel lifted and/or wheels tilted functionality
    STOP,       // Stop mower
    UNDEFINED   // This is used by foreign side to inform that it doesn't has a configuration for this sensor
};

#pragma pack(push, 1)
struct HallConfig {
    HallMode mode : 3;  // 1 bit reserved
    bool active_low : 1;
} __attribute__((packed));
#pragma pack(pop)

// For each active hall, we've a handle of it for quick and direct access
struct HallHandle {
    HallConfig config;
    std::function<bool()> get_value;
};

#define MAX_HALL_INPUTS 10  // How much Hall-inputs we support. 4 * OM + 6 * Stock-CoverUI + 0 spare (because not yet required to make it fixed)

// LL/HL config packet, bi-directional, flexible-length, with defaults for YF-C500.
#pragma pack(push, 1)
struct ll_high_level_config {
    // ATTENTION: This is a flexible length struct. It is allowed to grow independently to HL without loosing compatibility,
    //    but never change or restructure already published member, except you really know their consequences.

    // uint8_t type; Just for illustration. Get set in wire buffer with type PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ or PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP

    ConfigOptions options = {.dfp_is_5v = OptionState::OFF, .background_sounds = OptionState::OFF, .ignore_charging_current = OptionState::OFF};
    uint16_t rain_threshold = 700;         // If (stock CoverUI) rain value < rain_threshold then it rains. Expected to differ between C500, SA and SC types (0xFFFF = unknown)
    float v_charge_cutoff = 30.0f;         // Protective max. charging voltage before charging get switched off (-1 = unknown)
    float i_charge_cutoff = 1.5f;          // Protective max. charging current before charging get switched off (-1 = unknown)
    float v_battery_cutoff = 29.0f;        // Protective max. battery voltage before charging get switched off (-1 = unknown)
    float v_battery_empty = 21.7f + 0.3f;  // Empty battery voltage used for % calc of capacity (-1 = unknown)
    float v_battery_full = 28.7f - 0.3f;   // Full battery voltage used for % calc of capacity (-1 = unknown)
    uint16_t lift_period = 100;            // Period (ms) for >=2 wheels to be lifted in order to count as emergency (0 = disable, 0xFFFF = unknown). This is to filter uneven ground
    uint16_t tilt_period = 2500;           // Period (ms) for a single wheel to be lifted in order to count as emergency (0 = disable, 0xFFFF = unknown). This is to filter uneven ground
    uint8_t shutdown_esc_max_pitch = 0;    // Do not shutdown ESCs if absolute pitch angle is greater than this (0 = disable, 0xffff = unknown)
    iso639_1 language = {'e', 'n'};        // ISO 639-1 (2-char) language code (en, de, ...)
    uint8_t volume = 80;                   // Volume (0-100%) feedback (if directly changed i.e. via CoverUI or WebApp) (0xff = do not change)
    HallConfig hall_configs[MAX_HALL_INPUTS] = {
        {HallMode::LIFT_TILT, true},  // [0] OM Hall-1 input (Lift1) = YF-C500 default
        {HallMode::LIFT_TILT, true},  // [1] OM Hall-2 input (Lift2) = YF-C500 default
        {HallMode::STOP, true},       // [2] OM Hall-3 input (Stop1) = YF-C500 default
        {HallMode::STOP, true},       // [3] OM Hall-4 input (Stop2) = YF-C500 default
        //{HallMode::LIFT_TILT, false},  // [4] CoverUI-LIFT | LIFTX (up to CoverUI FW 2.0x)
        //{HallMode::LIFT_TILT, false},  // [5] CoverUI-LIFTX (as of CoverUI FW 2.1x)
        //{HallMode::LIFT_TILT, false},  // [6] CoverUI-LBUMP | RBUMP (up to CoverUI FW 2.0x)
        //{HallMode::LIFT_TILT, false},  // [7] CoverUI-RBUMP (as of CoverUI FW 2.1x)
        //{HallMode::STOP, false},       // [8] CoverUI-Stop1
        //{HallMode::STOP, false},       // [9] CoverUI-Stop2
    };
    // INFO: Before adding a new member here: Decide if and how much hall_configs spares do we like to have

    // uint16_t crc;  Just for illustration, that it get appended, but only within the wire buffer
} __attribute__((packed));
#pragma pack(pop)

#endif

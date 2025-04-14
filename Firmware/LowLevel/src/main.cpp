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
#include <Arduino.h>
#include <FastCRC.h>
#include <LittleFS.h>
#include <NeoPixelConnect.h>
#include <PacketSerial.h>

#include <etl/vector.h>

#include "datatypes.h"
#include "imu.h"
#include "pins.h"
#include "ui_board.h"
#include "debug.h"

#ifdef ENABLE_SOUND_MODULE
#include <soundsystem.h>
using namespace soundSystem;
#endif

#define IMU_CYCLETIME 20              // cycletime for refresh IMU data
#define STATUS_CYCLETIME 100          // cycletime for refresh analog and digital Statusvalues
#define UI_GET_VERSION_CYCLETIME 5000 // cycletime for UI Get_Version request (UI available check)
#define UI_GET_VERSION_TIMEOUT 100    // timeout for UI Get_Version response (UI available check)

#define BUTTON_EMERGENCY_MILLIS 20 // Time for button emergency to activate. This is to debounce the button.

#define PACKET_SERIAL Serial1
SerialPIO uiSerial(PIN_UI_TX, PIN_UI_RX, 250);

#define UI1_SERIAL uiSerial

// Millis after charging is retried
#define CHARGING_RETRY_MILLIS 10000

/**
 * @brief Some hardware parameters
 */
#define VIN_R1 10000.0f
#define VIN_R2 1000.0f
#define R_SHUNT 0.003f
#define CURRENT_SENSE_GAIN 100.0f

// Emergency will be engaged, if no heartbeat was received in this time frame.
#define HEARTBEAT_MILLIS 500

NeoPixelConnect p(PIN_NEOPIXEL, 1);
uint8_t led_blink_counter = 0;

PacketSerial packetSerial; // COBS communication PICO <> Raspi
PacketSerial UISerial;     // COBS communication PICO UI-Board
FastCRC16 CRC16;

unsigned long last_imu_millis = 0;
unsigned long last_status_update_millis = 0;
unsigned long last_heartbeat_millis = 0;
unsigned long next_ui_msg_millis = 0;

unsigned long lift_emergency_started = 0;
unsigned long tilt_emergency_started = 0;
unsigned long button_emergency_started = 0;

unsigned long ui_get_version_next_millis = 0;     // Next cycle when to check for a UI version
unsigned long ui_get_version_respond_timeout = 0; // When UI Get_Version response times out

// Stock UI
uint8_t stock_ui_emergency_state = 0; // Get set by received Get_Emergency packet
bool stock_ui_rain = false;           // Get set by received Get_Rain packet

// Predefined message buffers, so that we don't need to allocate new ones later.
struct ll_imu imu_message = {0};
struct ll_status status_message = {};
// current high level state
struct ll_high_level_state last_high_level_state = {0};

// Struct for the current LEDs. This gets sent to the UI periodically
struct msg_set_leds leds_message = {0};

// A mutex which is used by core1 each time status_message is modified.
// We can lock it during message transmission to prevent core1 to modify data in this time.
auto_init_mutex(mtx_status_message);

bool emergency_latch = true;
bool sound_available = false;
bool charging_allowed = false;
bool ROS_running = false;
unsigned long charging_disabled_time = 0;

float imu_temp[9];
float pitch_angle = 0, roll_angle = 0, tilt_angle = 0;

uint16_t ui_version = 0;                   // Last received UI firmware version
uint8_t ui_topic_bitmask = Topic_set_leds; // UI subscription, default to Set_LEDs
uint16_t ui_interval = 1000;               // UI send msg (LED/State) interval (ms)

// LL/HL config
#define CONFIG_FILENAME "/openmower.cfg"  // Where our llhl_config get saved in LittleFS (flash)
uint16_t config_crc_in_flash = 0;
uint16_t config_size_in_flash = 0;        // Require for wear level protection
struct ll_high_level_config llhl_config;  // LL/HL configuration (is initialized with YF-C500 defaults)

// Available hall input sources, same order as in ll_high_level_config.hall_configs
const std::function<bool()> available_halls[MAX_HALL_INPUTS] = {
    []() { return gpio_get(PIN_EMERGENCY_1); },                             // OM-Hall-1 (default Lift1)
    []() { return gpio_get(PIN_EMERGENCY_2); },                             // OM-Hall-2 (default Lift2)
    []() { return gpio_get(PIN_EMERGENCY_3); },                             // OM-Hall-3 (default Stop1)
    []() { return gpio_get(PIN_EMERGENCY_4); },                             // OM-Hall-4 (default Stop2)
    []() { return stock_ui_emergency_state & LL_EMERGENCY_BIT_CU_LIFT; },   // CoverUI-LIFT | LIFTX (up to CoverUI FW 2.0x)
    []() { return stock_ui_emergency_state & LL_EMERGENCY_BIT_CU_LIFTX; },  // CoverUI-LIFTX (as of CoverUI FW 2.1x)
    []() { return stock_ui_emergency_state & LL_EMERGENCY_BIT_CU_BUMP; },   // CoverUI-LBUMP | RBUMP (up to CoverUI FW 2.0x)
    []() { return stock_ui_emergency_state & LL_EMERGENCY_BIT_CU_RBUMP; },  // CoverUI-RBUMP (as of CoverUI FW 2.1x)
    []() { return stock_ui_emergency_state & LL_EMERGENCY_BIT_CU_STOP1; },  // CoverUI-Stop1
    []() { return stock_ui_emergency_state & LL_EMERGENCY_BIT_CU_STOP2; },  // CoverUI-Stop2
};
// Instead of iterating constantly over all available_halls, we use this compacted vector (which only contain the used halls)
etl::vector<HallHandle, MAX_HALL_INPUTS> halls;

void sendMessage(void *message, size_t size);
void sendUIMessage(void *message, size_t size);
void onPacketReceived(const uint8_t *buffer, size_t size);
void onUIPacketReceived(const uint8_t *buffer, size_t size);
void manageUISubscriptions();
void readConfigFromFlash();
void saveConfigToFlash(const uint8_t *t_buffer, const size_t t_size, const uint16_t t_crc);
void updateConfigInFlash();

void setRaspiPower(bool power) {
    // Update status bits in the status message
    status_message.status_bitmask = (status_message.status_bitmask & 0b11111101) | ((power & 0b1) << 1);
    digitalWrite(PIN_RASPI_POWER, power);
}

void updateEmergency() {
    if (millis() - last_heartbeat_millis > HEARTBEAT_MILLIS) {
        emergency_latch = true;
        ROS_running = false;
    }
    uint8_t last_emergency = status_message.emergency_bitmask & LL_EMERGENCY_BIT_LATCH;
    uint8_t emergency_state = 0;

    // Check all emergency inputs (halls)
    bool stop_pressed = false;
    int num_lifted = 0;
    for (const auto &hall : halls) {
        if (hall.get_value() ^ hall.config.active_low) {  // Get emergency input value and invert if low-active
            if (hall.config.mode == HallMode::STOP) {
                stop_pressed = true;
            } else if (hall.config.mode == HallMode::LIFT_TILT) {
                num_lifted++;
            }
            // Unlikely to happen, but there's no more emergency than stop and multiple-wheels lifted
            if (stop_pressed && num_lifted >= 2)
                break;
        }
    }

    // Handle emergency "Stop" buttons
    if (stop_pressed) {
        if (button_emergency_started == 0) {
            button_emergency_started = millis();  // Just pressed, store the timestamp for debouncing
        } else if (button_emergency_started > 0 && (millis() - button_emergency_started) >= BUTTON_EMERGENCY_MILLIS) {
            emergency_state |= LL_EMERGENCY_BIT_STOP;  // Debounced
        }
    } else {
        button_emergency_started = 0;  // Not pressed, reset the time
    }

    // Handle lifted (>=2 wheels are lifted)
    if (num_lifted >= 2) {
        if (lift_emergency_started == 0) lift_emergency_started = millis();  // If we just lifted, store the timestamp
    } else {
        lift_emergency_started = 0;  // Not lifted, reset the time
    }

    // Handle tilted (at least one wheel is lifted)
    if (num_lifted >= 1) {
        if (tilt_emergency_started == 0) tilt_emergency_started = millis();  // If we just tilted, store the timestamp
    } else {
        tilt_emergency_started = 0;  // Not tilted, reset the time
    }

    // Evaluate lift & tilt periods
    if ((llhl_config.lift_period > 0 && lift_emergency_started > 0 && (millis() - lift_emergency_started) >= llhl_config.lift_period) ||
        (llhl_config.tilt_period > 0 && tilt_emergency_started > 0 && (millis() - tilt_emergency_started) >= llhl_config.tilt_period)) {
        emergency_state |= LL_EMERGENCY_BIT_LIFT;
    }

    if (emergency_state || emergency_latch) {
        emergency_latch = true;
        emergency_state |= LL_EMERGENCY_BIT_LATCH;
    }

    status_message.emergency_bitmask = emergency_state;

    // If it's a new emergency, instantly send the message. This is to not spam the channel during emergencies.
    if (last_emergency != (emergency_state & LL_EMERGENCY_BIT_LATCH)) {
        sendMessage(&status_message, sizeof(struct ll_status));

        // Update UI instantly
        manageUISubscriptions();
    }
}

// Deals with the physical information and control the UI-LEDs und buzzer in dependency of voltage und current values
void manageUILEDS() {
    // Show Info Docking LED
    if ((status_message.charging_current > 0.80f) && (status_message.v_charge > 20.0f))
        setLed(leds_message, LED_CHARGING, LED_blink_fast);
    else if ((status_message.charging_current <= 0.80f) && (status_message.charging_current >= 0.15f) &&
             (status_message.v_charge > 20.0f))
        setLed(leds_message, LED_CHARGING, LED_blink_slow);
    else if ((status_message.charging_current < 0.15f) && (status_message.v_charge > 20.0f))
        setLed(leds_message, LED_CHARGING, LED_on);
    else
        setLed(leds_message, LED_CHARGING, LED_off);

    // Show Info Battery state
    if (status_message.v_battery >= (llhl_config.v_battery_empty + 2.0f))
        setLed(leds_message, LED_BATTERY_LOW, LED_off);
    else
        setLed(leds_message, LED_BATTERY_LOW, LED_on);

    if (status_message.v_charge < 10.0f) // activate only when undocked
    {
        // use the first LED row as bargraph
        setBars7(leds_message, status_message.batt_percentage / 100.0);
        if (last_high_level_state.gps_quality == 0) {
            // if quality is 0, flash all LEDs to notify the user to calibrate.
            setBars4(leds_message, -1.0);
        } else {
            setBars4(leds_message, last_high_level_state.gps_quality / 100.0);
        }
    } else {
        setBars7(leds_message, 0);
        setBars4(leds_message, 0);
    }

    if (last_high_level_state.gps_quality < 25) {
        setLed(leds_message, LED_POOR_GPS, LED_on);
    } else if (last_high_level_state.gps_quality < 50) {
        setLed(leds_message, LED_POOR_GPS, LED_blink_fast);
    } else if (last_high_level_state.gps_quality < 75) {
        setLed(leds_message, LED_POOR_GPS, LED_blink_slow);
    } else {
        setLed(leds_message, LED_POOR_GPS, LED_off);
    }

    // Let S1 show if ros is connected and which state it's in
    if (!ROS_running) {
        setLed(leds_message, LED_S1, LED_off);
    } else {
        switch (HighLevelState::getMode(last_high_level_state.current_mode)) {
            case HighLevelState::Mode::IDLE:
                setLed(leds_message, LED_S1, LED_on);
                break;
            case HighLevelState::Mode::AUTONOMOUS:
                setLed(leds_message, LED_S1, LED_blink_slow);
                break;
            default:
                setLed(leds_message, LED_S1, LED_blink_fast);
                break;
        }
        switch (HighLevelState::getSubMode(last_high_level_state.current_mode)) {
            case 1:  // Docking or Record outline
                setLed(leds_message, LED_S2, LED_blink_slow);
                break;
            case 2:  // Undocking or Record obstacle
                setLed(leds_message, LED_S2, LED_blink_fast);
                break;
            case 3:  // Not defined yet
                setLed(leds_message, LED_S2, LED_on);
                break;
            default:
                setLed(leds_message, LED_S2, LED_off);
                break;
        }
    }

    // Show Info mower lifted or stop button pressed
    if (status_message.emergency_bitmask & LL_EMERGENCY_BIT_STOP) {
        setLed(leds_message, LED_MOWER_LIFTED, LED_blink_fast);
    } else if (status_message.emergency_bitmask & LL_EMERGENCY_BIT_LIFT) {
        setLed(leds_message, LED_MOWER_LIFTED, LED_blink_slow);
    } else if (status_message.emergency_bitmask & LL_EMERGENCY_BIT_LATCH) {
        setLed(leds_message, LED_MOWER_LIFTED, LED_on);
    } else {
        setLed(leds_message, LED_MOWER_LIFTED, LED_off);
    }

    sendUIMessage(&leds_message, sizeof(leds_message));
}

// Manage send status to UI, dependent on ui_topic_bitmask (subscription)
void manageUISubscriptions()
{
    if (ui_topic_bitmask & Topic_set_leds)
    {
        manageUILEDS();
    }

    if (ui_topic_bitmask & Topic_set_ll_status)
    {
        sendUIMessage(&status_message, sizeof(struct ll_status));
    }

    if (ui_topic_bitmask & Topic_set_hl_state)
    {
        sendUIMessage(&last_high_level_state, sizeof(struct ll_high_level_state));
    }
}

void setup1() {
    pinMode(PIN_MUX_OUT, OUTPUT);
    pinMode(PIN_MUX_ADDRESS_0, OUTPUT);
    pinMode(PIN_MUX_ADDRESS_1, OUTPUT);
    pinMode(PIN_MUX_ADDRESS_2, OUTPUT);
}

void loop1() {
    // Loop through the mux and query actions. Store the result in the multicore fifo
    for (uint8_t mux_address = 0; mux_address < 7; mux_address++) {
        gpio_put_masked(0b111 << 13, mux_address << 13);
        delay(1);
        bool state = gpio_get(PIN_MUX_IN);

        switch (mux_address) {
            case 5:
                mutex_enter_blocking(&mtx_status_message);

                if (state || stock_ui_rain) {
                    status_message.status_bitmask |= LL_STATUS_BIT_RAIN;
                } else {
                    status_message.status_bitmask &= ~LL_STATUS_BIT_RAIN;
                }
                mutex_exit(&mtx_status_message);

                break;
            case 6:
                mutex_enter_blocking(&mtx_status_message);
                if (state) {
                    status_message.status_bitmask &= ~LL_STATUS_BIT_SOUND_BUSY;
                } else {
                    status_message.status_bitmask |= LL_STATUS_BIT_SOUND_BUSY;
                }
                mutex_exit(&mtx_status_message);
                break;
            default:
                break;
        }
    }

#ifdef ENABLE_SOUND_MODULE
    soundSystem::processSounds(status_message, ROS_running, last_high_level_state);
#endif

    delay(100);
}

void setup() {
    //  We do hardware init in this core, so that we don't get invalid states.
    //  Therefore, we pause the other core until vars used in OtherCore got initialized.
    rp2040.idleOtherCore();

    DEBUG_BEGIN(9600);

    emergency_latch = true;
    ROS_running = false;

    lift_emergency_started = 0;
    button_emergency_started = 0;
    // Initialize messages
    imu_message = {0};
    status_message = {0};
    imu_message.type = PACKET_ID_LL_IMU;
    status_message.type = PACKET_ID_LL_STATUS;

    // Save to start other core now, as well as required i.e. for LittleFS
    rp2040.resumeOtherCore();

    // Setup pins
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_ENABLE_CHARGE, OUTPUT);
    digitalWrite(PIN_ENABLE_CHARGE, HIGH);

    pinMode(PIN_ESC_SHUTDOWN, OUTPUT);
    digitalWrite(PIN_ESC_SHUTDOWN, LOW);

    gpio_init(PIN_RASPI_POWER);
    gpio_put(PIN_RASPI_POWER, true);
    gpio_set_dir(PIN_RASPI_POWER, true);
    gpio_put(PIN_RASPI_POWER, true);

    // Enable raspi power
    p.neoPixelSetValue(0, 32, 0, 0, true);
    delay(1000);
    setRaspiPower(true);
    p.neoPixelSetValue(0, 255, 0, 0, true);

    pinMode(PIN_EMERGENCY_1, INPUT);
    pinMode(PIN_EMERGENCY_2, INPUT);
    pinMode(PIN_EMERGENCY_3, INPUT);
    pinMode(PIN_EMERGENCY_4, INPUT);

    analogReadResolution(12);

    // Init serial com to RasPi
    PACKET_SERIAL.begin(115200);
    packetSerial.setStream(&PACKET_SERIAL);
    packetSerial.setPacketHandler(&onPacketReceived);

    // Init serial com to CoverUI
    UI1_SERIAL.begin(115200);
    UISerial.setStream(&UI1_SERIAL);
    UISerial.setPacketHandler(&onUIPacketReceived);

    // Initialize flash and try to read config.
    // ATTENTION: LittleFS needs other core (at least for initial format)!
    LittleFS.begin();
    readConfigFromFlash();  // Set llhl_config with defaults or saved state

#ifdef ENABLE_SOUND_MODULE
    p.neoPixelSetValue(0, 0, 255, 255, true);
    sound_available = soundSystem::begin();
    if (sound_available) {
        p.neoPixelSetValue(0, 0, 0, 255, true);  // Blue
        soundSystem::applyConfig(llhl_config, true);
        // Do NOT play any initial sound now, because we've to handle the special case of
        // old DFPlayer SD-Card format @ DFROBOT LISP3 with wrong IO2 level. See soundSystem::processSounds()
        p.neoPixelSetValue(0, 255, 255, 0, true);
    } else {
        for (uint8_t b = 0; b < 3; b++) {
            p.neoPixelSetValue(0, 0, 0, 0, true);
            delay(200);
            p.neoPixelSetValue(0, 0, 0, 255, true);
            delay(200);
        }
    }
#endif

    /*
     * IMU INITIALIZATION
     */
    bool init_imu_success = false;
    int init_imu_tries = 10;
    while(init_imu_tries --> 0) {
        if(init_imu()) {
            init_imu_success = true;
            break;
        }
        DEBUG_PRINTF("IMU initialization unsuccessful, retrying in 1 sec (%d tries left)\n", init_imu_tries + 1);

        p.neoPixelSetValue(0, 0, 0, 0, true);
        delay(800);
        p.neoPixelSetValue(255, 255, 0, 0, true);
        delay(100);
        p.neoPixelSetValue(0, 0, 0, 0, true);
        delay(100);
    }

    if (!init_imu_success) {
        unsigned long next_ann = millis();
        DEBUG_PRINTLN("IMU initialization failed");
        DEBUG_PRINTLN("Check IMU wiring or try cycling power");

        status_message.status_bitmask = 0;
        while (1) {  // Infinite blink RED for IMU failure
            p.neoPixelSetValue(0, 255, 0, 0, true);
            delay(500);
            p.neoPixelSetValue(0, 0, 0, 0, true);
            delay(500);
#ifdef ENABLE_SOUND_MODULE
            if (millis() >= next_ann) {
                soundSystem::playSound(SOUND_TRACK_ADV_IMU_INIT_FAILED);
                soundSystem::playSound(SOUND_TRACK_BGD_OM_ALARM);
                next_ann = millis() + 8000;
            }
#endif
        }
    }
    p.neoPixelSetValue(0, 255, 255, 255, true);  // White for IMU Success
    DEBUG_PRINTLN("Imu initialized");

    status_message.status_bitmask |= LL_STATUS_BIT_INITIALIZED;

    // Cover UI board clear all LEDs
    leds_message.type = Set_LEDs;
    leds_message.leds = 0;
    sendUIMessage(&leds_message, sizeof(leds_message));

    p.neoPixelSetValue(0, 255, 255, 255, true);  // White 1s final success
    delay(1000);

    digitalWrite(LED_BUILTIN, HIGH);  // Signal that both cores got setup and looping start
}

void onUIPacketReceived(const uint8_t *buffer, size_t size) {

    u_int16_t *crc_pointer = (uint16_t *) (buffer + (size - 2));
    u_int16_t readcrc = *crc_pointer;

    // check structure size
    if (size < 4)
        return;

    // check the CRC
    uint16_t crc = CRC16.ccitt(buffer, size - 2);

    if (buffer[size - 1] != ((crc >> 8) & 0xFF) ||
        buffer[size - 2] != (crc & 0xFF))
        return;

    if (buffer[0] == Get_Version && size == sizeof(struct msg_get_version))
    {
        struct msg_get_version *msg = (struct msg_get_version *)buffer;
        ui_version = msg->version;
        status_message.status_bitmask |= LL_STATUS_BIT_UI_AVAIL;
        ui_get_version_respond_timeout = 0;
    }
    else if (buffer[0] == Get_Button && size == sizeof(struct msg_event_button))
    {
        struct msg_event_button *msg = (struct msg_event_button *)buffer;
        struct ll_ui_event ui_event;
        ui_event.type = PACKET_ID_LL_UI_EVENT;
        ui_event.button_id = msg->button_id;
        ui_event.press_duration = msg->press_duration;
        sendMessage(&ui_event, sizeof(ui_event));

#ifdef ENABLE_SOUND_MODULE
        // Handle Sound Buttons. FIXME: Should/might go to mower_logic? But as sound isn't hip ... :-/
        switch (msg->button_id) {
            case 8:  // Mon = Volume up
                llhl_config.volume = soundSystem::setVolumeUp();
                updateConfigInFlash();
                break;
            case 9:  // Tue = Volume down
                llhl_config.volume = soundSystem::setVolumeDown();
                updateConfigInFlash();
                break;

            default:
                break;
        }
#endif
    } else if (buffer[0] == Get_Emergency && size == sizeof(struct msg_event_emergency)) {
        struct msg_event_emergency *msg = (struct msg_event_emergency *)buffer;
        stock_ui_emergency_state = msg->state;
    } else if (buffer[0] == Get_Rain && size == sizeof(struct msg_event_rain)) {
        struct msg_event_rain *msg = (struct msg_event_rain *)buffer;
        stock_ui_rain = (msg->value < llhl_config.rain_threshold);
    } else if (buffer[0] == Get_Subscribe && size == sizeof(struct msg_event_subscribe)) {
        struct msg_event_subscribe *msg = (struct msg_event_subscribe *)buffer;
        ui_topic_bitmask = msg->topic_bitmask;
        ui_interval = msg->interval;
    }
}

void sendConfigMessage(const uint8_t pkt_type) {
    size_t msg_size = sizeof(struct ll_high_level_config) + 3;  // + 1 type + 2 crc
    uint8_t *msg = (uint8_t *)malloc(msg_size);
    if (msg == NULL)
        return;
    msg[0] = pkt_type;
    memcpy(msg + 1, &llhl_config, sizeof(struct ll_high_level_config));  // Copy our live config into the message, behind type
    sendMessage(msg, msg_size);                                          // sendMessage() also calculate the packet CRC
    free(msg);
}

/**
 * @brief applyConfig applies those members who are not undefined/unknown.
 * This function get called either when receiving a ll_high_level_config packet from HL, or during boot after read from LittleFS
 * @param buffer
 * @param size of buffer (without packet type nor CRC)
 */
void applyConfig(const uint8_t *buffer, const size_t size) {
    // This is a flexible length packet where the size may vary when ll_high_level_config struct get enhanced only on one side.
    // If payload size is larger than our struct size, ensure that we only copy those we know of (which is our struct size).
    // If payload size is smaller than our struct size, copy only the payload we got, but ensure that the unsent member(s) have reasonable defaults.
    size_t payload_size = min(sizeof(ll_high_level_config), size);

    // Use a temporary rcv_config for easier member access.
    // If payload is smaller (older), struct already contains reasonable defaults.
    ll_high_level_config rcv_config;
    memcpy(&rcv_config, buffer, payload_size);  // Copy payload to temporary config

    // Use a temporary new_config for save sanity adaption before copy to our live config.
    // It's important to use a clean one with LL-default values, and not the current live llhl_config one,
    // because we need to ensure that re-commented mower_config parameter get the LL-default value and not the last saved one!
    ll_high_level_config new_config;

    // HL always force language
    strncpy(new_config.language, rcv_config.language, 2);

    // Take over only those (HL) values which are not "undefined/unknown"
    if (rcv_config.options.dfp_is_5v != OptionState::UNDEFINED) new_config.options.dfp_is_5v = rcv_config.options.dfp_is_5v;
    if (rcv_config.options.background_sounds != OptionState::UNDEFINED) new_config.options.background_sounds = rcv_config.options.background_sounds;
    if (rcv_config.options.ignore_charging_current != OptionState::UNDEFINED) new_config.options.ignore_charging_current = rcv_config.options.ignore_charging_current;
    if (rcv_config.rain_threshold != 0xffff) new_config.rain_threshold = rcv_config.rain_threshold;
    if (rcv_config.v_charge_cutoff >= 0) new_config.v_charge_cutoff = min(rcv_config.v_charge_cutoff, 36.0f);  // Rated max. limited by MAX20405
    if (rcv_config.i_charge_cutoff >= 0) new_config.i_charge_cutoff = min(rcv_config.i_charge_cutoff, 5.0f);   // Absolute max. limited by D2/D3 Schottky
    if (rcv_config.v_battery_cutoff >= 0) new_config.v_battery_cutoff = rcv_config.v_battery_cutoff;
    if (rcv_config.v_battery_empty >= 0) new_config.v_battery_empty = rcv_config.v_battery_empty;
    if (rcv_config.v_battery_full >= 0) new_config.v_battery_full = rcv_config.v_battery_full;
    if (rcv_config.lift_period != 0xffff) new_config.lift_period = rcv_config.lift_period;
    if (rcv_config.tilt_period != 0xffff) new_config.tilt_period = rcv_config.tilt_period;
    if (rcv_config.shutdown_esc_max_pitch != 0xff) new_config.shutdown_esc_max_pitch = rcv_config.shutdown_esc_max_pitch;
    if (rcv_config.volume != 0xff) new_config.volume = rcv_config.volume;

    // Handle all emergency/halls
    halls.clear();
    for (size_t i = 0; i < MAX_HALL_INPUTS; i++) {
        // Take over those halls which are not "undefined"
        if (rcv_config.hall_configs[i].mode != HallMode::UNDEFINED)
            new_config.hall_configs[i] = rcv_config.hall_configs[i];

        if (new_config.hall_configs[i].mode == HallMode::OFF)
            continue;

        // Apply used hall to our compacted stop/lift-hall vector which get used by updateEmergency()
        halls.push_back({new_config.hall_configs[i], available_halls[i]});
    }
    llhl_config = new_config;  // Make new config live
}

void onPacketReceived(const uint8_t *buffer, const size_t size) {
    // sanity check for CRC to work (1 type, 1 data, 2 CRC)
    if (size < 4)
        return;

    // check the CRC
    uint16_t crc = CRC16.ccitt(buffer, size - 2);

    if (buffer[size - 1] != ((crc >> 8) & 0xFF) ||
        buffer[size - 2] != (crc & 0xFF))
        return;

    if (buffer[0] == PACKET_ID_LL_HEARTBEAT && size == sizeof(struct ll_heartbeat)) {

        // CRC and packet is OK, reset watchdog
        last_heartbeat_millis = millis();
        struct ll_heartbeat *heartbeat = (struct ll_heartbeat *) buffer;
        if (heartbeat->emergency_release_requested) {
            emergency_latch = false;
        }
        // Check in this order, so we can set it again in the same packet if required.
        if (heartbeat->emergency_requested) {
            emergency_latch = true;
        }
        if (!ROS_running) {
            // ROS is running (again (i.e. due to restart after reconfiguration))
            ROS_running = true;
        }
    } else if (buffer[0] == PACKET_ID_LL_HIGH_LEVEL_STATE && size == sizeof(struct ll_high_level_state)) {
        // copy the state
        last_high_level_state = *((struct ll_high_level_state *) buffer);
    } else if (buffer[0] == PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ || buffer[0] == PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP) {
        applyConfig(buffer + 1, size - 3);  // Skip packet- type and CRC

#ifdef ENABLE_SOUND_MODULE
        // We can't move the sound stuff to applyConfig because applyConfig get also called by readConfigFromFlash()
        // which is before (and also needs to be before) soundSystem::begin()
        soundSystem::applyConfig(llhl_config, true);
#endif

        // Response if requested (before save, to ensure REQ/RSP timing)
        if (buffer[0] == PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ)
            sendConfigMessage(PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP);  // Other side requested a config response

        saveConfigToFlash(buffer, size, crc);
    }
}

// returns true, if it's a good idea to charge the battery (current, voltages, ...)
bool checkShouldCharge() {
    return status_message.v_charge < llhl_config.v_charge_cutoff && status_message.charging_current < llhl_config.i_charge_cutoff && status_message.v_battery < llhl_config.v_battery_cutoff;
}

void updateChargingEnabled() {
    // Always enable for regen when not docked
    if (status_message.v_charge < 3.0f) {
        digitalWrite(PIN_ENABLE_CHARGE, HIGH);
        charging_allowed = false; // For high level status
        return;
    }
    if (charging_allowed) {
        if (!checkShouldCharge()) {
            digitalWrite(PIN_ENABLE_CHARGE, LOW);
            charging_allowed = false;
            charging_disabled_time = millis();
        }
    } else {
        // enable charging after CHARGING_RETRY_MILLIS
        if (millis() - charging_disabled_time > CHARGING_RETRY_MILLIS) {
            if (!checkShouldCharge()) {
                digitalWrite(PIN_ENABLE_CHARGE, LOW);
                charging_allowed = false;
                charging_disabled_time = millis();
            } else {
                digitalWrite(PIN_ENABLE_CHARGE, HIGH);
                charging_allowed = true;
            }
        }
    }
}

void updateNeopixel() {
    led_blink_counter++;

    if (emergency_latch && led_blink_counter & 0b100) {  // slow blink on emergencies
        p.neoPixelSetValue(0, 128, 0, 0, true);          // 1/2 red
    } else {
        if (ROS_running) {
            p.neoPixelSetValue(0, 0, 255, 0, true);  // green
        } else {
            p.neoPixelSetValue(0, 255, 50, 0, true);  // yellow
        }
#if defined(WT901) || defined(WT901_INSTEAD_OF_SOUND)
        if (led_blink_counter & 0b10 && imu_comms_error()) {  // fast blink on communication error (condition order matters -> short-circuit evaluation)
            p.neoPixelSetValue(0, 255, 0, 255, true);         // magenta
        }
#endif
    }
}

void loop() {
    packetSerial.update();
    UISerial.update();
    imu_loop();
    updateChargingEnabled();
    updateEmergency();

    unsigned long now = millis();
    if (now - last_imu_millis > IMU_CYCLETIME) {
        // we have to copy to the temp data structure due to alignment issues
        imu_read(imu_temp, imu_temp + 3, imu_temp + 6);
        imu_message.acceleration_mss[0] = imu_temp[0];
        imu_message.acceleration_mss[1] = imu_temp[1];
        imu_message.acceleration_mss[2] = imu_temp[2];
        imu_message.gyro_rads[0] = imu_temp[3];
        imu_message.gyro_rads[1] = imu_temp[4];
        imu_message.gyro_rads[2] = imu_temp[5];
        imu_message.mag_uT[0] = imu_temp[6];
        imu_message.mag_uT[1] = imu_temp[7];
        imu_message.mag_uT[2] = imu_temp[8];

        imu_message.dt_millis = now - last_imu_millis;
        sendMessage(&imu_message, sizeof(struct ll_imu));

        last_imu_millis = now;

        // Update pitch, roll, tilt
        pitch_angle = atan2f(imu_temp[0], imu_temp[2]) * 180.0f / M_PI;
        roll_angle = atan2f(imu_temp[1], imu_temp[2]) * 180.0f / M_PI;
        float accXY = sqrtf((imu_temp[0]*imu_temp[0]) + (imu_temp[1]*imu_temp[1]));
        tilt_angle = atan2f(accXY, imu_temp[2]) * 180.0f / M_PI;
    }

    if (now - last_status_update_millis > STATUS_CYCLETIME) {
        updateNeopixel();

        status_message.v_battery =
                (float) analogRead(PIN_ANALOG_BATTERY_VOLTAGE) * (3.3f / 4096.0f) * ((VIN_R1 + VIN_R2) / VIN_R2);
        status_message.v_charge =
                (float) analogRead(PIN_ANALOG_CHARGE_VOLTAGE) * (3.3f / 4096.0f) * ((VIN_R1 + VIN_R2) / VIN_R2);
        status_message.charging_current = llhl_config.options.ignore_charging_current == OptionState::ON ? -1.0f : (float)analogRead(PIN_ANALOG_CHARGE_CURRENT) * (3.3f / 4096.0f) / (CURRENT_SENSE_GAIN * R_SHUNT);


        // ESC power saving
        bool shutdown_escs = false;
        if (llhl_config.shutdown_esc_max_pitch != 0) {
            bool emergency_or_idle = emergency_latch || HighLevelState::getMode(last_high_level_state.current_mode) == HighLevelState::Mode::IDLE;
            bool on_slope = fabs(pitch_angle) > llhl_config.shutdown_esc_max_pitch;
            if(ROS_running && emergency_or_idle && !on_slope) {
                shutdown_escs = true;
            }
        }
        if (shutdown_escs){
            digitalWrite(PIN_ESC_SHUTDOWN, HIGH);
            status_message.status_bitmask &= 0b11110111;
        } else {
            digitalWrite(PIN_ESC_SHUTDOWN, LOW);
            status_message.status_bitmask |= 0b1000;
        }


        status_message.status_bitmask = (status_message.status_bitmask & 0b11111011) | ((charging_allowed & 0b1) << 2);
        status_message.status_bitmask = (status_message.status_bitmask & 0b11011111) | ((sound_available & 0b1) << 5);

        // calculate percent value accu filling
        float delta = llhl_config.v_battery_full - llhl_config.v_battery_empty;
        float vo = status_message.v_battery - llhl_config.v_battery_empty;
        status_message.batt_percentage = vo / delta * 100;
        if (status_message.batt_percentage > 100)
            status_message.batt_percentage = 100;

        mutex_enter_blocking(&mtx_status_message);
        sendMessage(&status_message, sizeof(struct ll_status));
        mutex_exit(&mtx_status_message);

        last_status_update_millis = now;
#ifdef USB_DEBUG
        DEBUG_SERIAL.print("status: 0b");
        DEBUG_SERIAL.print(status_message.status_bitmask, BIN);
        DEBUG_SERIAL.print("\t");

        DEBUG_SERIAL.print("vin: ");
        DEBUG_SERIAL.print(status_message.v_battery, 3);
        DEBUG_SERIAL.print(" V\t");
        DEBUG_SERIAL.print("vcharge: ");
        DEBUG_SERIAL.print(status_message.v_charge, 3);
        DEBUG_SERIAL.print(" V\t");
        DEBUG_SERIAL.print("charge_current: ");
        DEBUG_SERIAL.print(status_message.charging_current, 3);
        DEBUG_SERIAL.print(" A\t");
        DEBUG_SERIAL.print("emergency: 0b");
        DEBUG_SERIAL.print(status_message.emergency_bitmask, BIN);
        DEBUG_SERIAL.println();
#endif
    }

    if (now > next_ui_msg_millis)
    {
        next_ui_msg_millis = now + ui_interval;
        manageUISubscriptions();
    }

    // Check UI version/available
    if (ui_get_version_respond_timeout && now > ui_get_version_respond_timeout)
    {
        status_message.status_bitmask &= ~LL_STATUS_BIT_UI_AVAIL;
        ui_version = 0;
        ui_get_version_respond_timeout = 0;
        stock_ui_emergency_state = 0; // Ensure that a stock-emergency state doesn't remain active if the UI got unplugged
    }
    if (now > ui_get_version_next_millis)
    {
        ui_get_version_next_millis = now + UI_GET_VERSION_CYCLETIME;
        ui_get_version_respond_timeout = now + UI_GET_VERSION_TIMEOUT;
        struct msg_get_version msg;
        msg.type = Get_Version;
        sendUIMessage(&msg, sizeof(msg));
    }
}

void sendMessage(void *message, size_t size) {
    // Only send messages, if ROS is running, else Raspi sometimes doesn't boot
    if (!ROS_running)
        return;

    // packages need to be at least 1 byte of type, 1 byte of data and 2 bytes of CRC
    if (size < 4) {
        return;
    }
    uint8_t *data_pointer = (uint8_t *) message;

    // calculate the CRC
    uint16_t crc = CRC16.ccitt((uint8_t *) message, size - 2);
    data_pointer[size - 1] = (crc >> 8) & 0xFF;
    data_pointer[size - 2] = crc & 0xFF;

    packetSerial.send(data_pointer, size);
}

void sendUIMessage(void *message, size_t size) {
    // packages need to be at least 1 byte of type, 1 byte of data and 2 bytes of CRC
    if (size < 4) {
        return;
    }
    uint8_t *data_pointer = (uint8_t *) message;

    // calculate the CRC
    uint16_t crc = CRC16.ccitt((uint8_t *) message, size - 2);
    data_pointer[size - 1] = (crc >> 8) & 0xFF;
    data_pointer[size - 2] = crc & 0xFF;

    UISerial.send((uint8_t *) message, size);
}

void readConfigFromFlash() {
    File f = LittleFS.open(CONFIG_FILENAME, "r");
    if (!f) return;

    // sanity check for CRC to work (1 type, 1 data, 2 CRC)
    const size_t size = f.size();
    if (size < 4) {
        f.close();
        return;
    }

    // read config
    uint8_t *buffer = (uint8_t *)malloc(size);
    if (buffer == NULL) return;

    f.read(buffer, size);
    f.close();

    // check the CRC
    uint16_t crc = CRC16.ccitt(buffer, size - 2);

    if (buffer[size - 1] != ((crc >> 8) & 0xFF) ||
        buffer[size - 2] != (crc & 0xFF))
        return;

    config_crc_in_flash = crc;
    applyConfig(buffer + 1, size - 3);  // Skip Type & CRC
    free(buffer);
}

/**
 * @brief saveConfigToFlash() save a (received) config packet to flash, if the crc differ to the one already stored in flash (wear level protection)
 *
 * We do store the whole received packet in flash and not the live (applied) config because this has the following advantages:
 * - We're free to change ll_high_level_config defaults in future without fiddling with already stored ones
 * - Reuse the already calculated and validated CRC
 *
 * @param t_buffer
 * @param t_size
 * @param t_crc
 */
void saveConfigToFlash(const uint8_t *t_buffer, const size_t t_size, const uint16_t t_crc) {
    if (t_crc == config_crc_in_flash) return;  // Protect wear leveling
    File f = LittleFS.open(CONFIG_FILENAME, "w");
    if (!f) return;
    if (f.write(t_buffer, t_size) == t_size) {
        config_crc_in_flash = t_crc;
        config_size_in_flash = t_size;
    }
    f.close();
}

/**
 * @brief updateConfigInFlash() will update the config in flash with the live one, if CRC differ (wear level protection)
 *
 */
void updateConfigInFlash() {
    // We need to simulate a packet
    uint16_t size = sizeof(struct ll_high_level_config) + 3;  // + 1 type + 2 crc
    if (config_size_in_flash > 3)
        size = min(size, config_size_in_flash);  // Wear level protection

    uint8_t *buffer = (uint8_t *)malloc(size);
    if (buffer == NULL) return;

    buffer[0] = PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP;  // Not needed in real, but let's pretend to be
    memcpy(buffer + 1, &llhl_config, size);          // Copy our live llhl_config into the buffer

    uint16_t crc = CRC16.ccitt(buffer, size - 2);
    if (crc == config_crc_in_flash) return;  // No need to write, protect wear leveling

    buffer[size - 1] = (crc >> 8) & 0xFF;
    buffer[size - 2] = crc & 0xFF;

    File f = LittleFS.open(CONFIG_FILENAME, "w");
    if (!f) return;
    if (f.write(buffer, size) == size) config_crc_in_flash = crc;
    f.close();
}

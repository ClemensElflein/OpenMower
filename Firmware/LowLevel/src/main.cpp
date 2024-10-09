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
#include <NeoPixelConnect.h>
#include <Arduino.h>
#include <FastCRC.h>
#include <PacketSerial.h>
#include "datatypes.h"
#include "config.h"
#include "pins.h"
#include "ui_board.h"
#include "imu.h"
#include "debug.h"
#include "nv_config.h"

#ifdef ENABLE_SOUND_MODULE
#include <soundsystem.h>
#endif

#define IMU_CYCLETIME 20              // cycletime for refresh IMU data
#define STATUS_CYCLETIME 100          // cycletime for refresh analog and digital Statusvalues
#define UI_GET_VERSION_CYCLETIME 5000 // cycletime for UI Get_Version request (UI available check)
#define UI_GET_VERSION_TIMEOUT 100    // timeout for UI Get_Version response (UI available check)

#define BUTTON_EMERGENCY_MILLIS 20 // Time for button emergency to activate. This is to debounce the button.

// Define to stream debugging messages via USB
// #define USB_DEBUG

// Only define DEBUG_SERIAL if USB_DEBUG is actually enabled.
// This enforces compile errors if it's used incorrectly.
#ifdef USB_DEBUG
#define DEBUG_SERIAL Serial
#endif
#define PACKET_SERIAL Serial1
SerialPIO uiSerial(PIN_UI_TX, PIN_UI_RX, 250);

#define UI1_SERIAL uiSerial

#define ANZ_SOUND_SD_FILES 3

// Millis after charging is retried
#define CHARGING_RETRY_MILLIS 10000

/**
 * @brief Some hardware parameters
 */
#define VIN_R1 10000.0f
#define VIN_R2 1000.0f
#define R_SHUNT 0.003f
#define CURRENT_SENSE_GAIN 100.0f

#define BATT_ABS_MAX 28.7f
#define BATT_ABS_Min 21.7f

#define BATT_FULL BATT_ABS_MAX - 0.3f
#define BATT_EMPTY BATT_ABS_Min + 0.3f

// Emergency will be engaged, if no heartbeat was received in this time frame.
#define HEARTBEAT_MILLIS 500

NeoPixelConnect p(PIN_NEOPIXEL, 1);
uint8_t led_blink_counter = 0;

PacketSerial packetSerial; // COBS communication PICO <> Raspi
PacketSerial UISerial;     // COBS communication PICO UI-Board
FastCRC16 CRC16;

#ifdef ENABLE_SOUND_MODULE
MP3Sound my_sound; // Soundsystem
#endif

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
struct ll_status status_message = {0};
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

uint16_t ui_version = 0;                   // Last received UI firmware version
uint8_t ui_topic_bitmask = Topic_set_leds; // UI subscription, default to Set_LEDs
uint16_t ui_interval = 1000;               // UI send msg (LED/State) interval (ms)

struct ll_high_level_config llhl_config;  // LL/HL configuration (get initialized with YF-C500 defaults)
nv_config::Config *nv_cfg;                // Non-volatile configuration

// Some vars related to PACKET_ID_LL_HIGH_LEVEL_CONFIG_*
uint8_t comms_version = 0;  // comms packet version (>0 if implemented)
uint8_t config_bitmask = 0; // See LL_HIGH_LEVEL_CONFIG_BIT_*

void sendMessage(void *message, size_t size);
void sendUIMessage(void *message, size_t size);
void onPacketReceived(const uint8_t *buffer, size_t size);
void onUIPacketReceived(const uint8_t *buffer, size_t size);
void manageUISubscriptions();

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

    // Read & assign emergencies in the same manner as in ll_status.emergency_bitmask
    uint8_t emergency_read = !gpio_get(PIN_EMERGENCY_3) << 1 | // Stop1
                             !gpio_get(PIN_EMERGENCY_4) << 2 | // Stop2
                             !gpio_get(PIN_EMERGENCY_1) << 3 | // Lift1
                             !gpio_get(PIN_EMERGENCY_2) << 4 | // Lift2
                             stock_ui_emergency_state; // OR with StockUI emergency
    uint8_t emergency_state = 0;

    // Some mowers have the logic level inverted, fix this...
    emergency_read = emergency_read ^ (LL_EMERGENCY_BIT_LIFT1 * LIFT1_IS_INVERTED);
    emergency_read = emergency_read ^ (LL_EMERGENCY_BIT_LIFT2 * LIFT2_IS_INVERTED);

    // Handle emergency "Stop" buttons
    if (emergency_read & LL_EMERGENCY_BITS_STOP) {
        // If we just pressed, store the timestamp
        if (button_emergency_started == 0) {
            button_emergency_started = millis();
        }
    } else {
        // Not pressed, reset the time
        button_emergency_started = 0;
    }

    if (button_emergency_started > 0 && (millis() - button_emergency_started) >= BUTTON_EMERGENCY_MILLIS)
    {
        emergency_state |= (emergency_read & LL_EMERGENCY_BITS_STOP);
    }

    // Handle lifted (both wheels are lifted)
    if ((emergency_read & LL_EMERGENCY_BITS_LIFT) == LL_EMERGENCY_BITS_LIFT) {
        // If we just lifted, store the timestamp
        if (lift_emergency_started == 0) {
            lift_emergency_started = millis();
        }
    } else {
        // Not lifted, reset the time
        lift_emergency_started = 0;
    }

    // Handle tilted (one wheel is lifted)
    if (emergency_read & LL_EMERGENCY_BITS_LIFT) {
        // If we just tilted, store the timestamp
        if (tilt_emergency_started == 0) {
            tilt_emergency_started = millis();
        }
    } else {
        // Not tilted, reset the time
        tilt_emergency_started = 0;
    }

    if ((LIFT_EMERGENCY_MILLIS > 0 && lift_emergency_started > 0 && (millis() - lift_emergency_started) >= LIFT_EMERGENCY_MILLIS) ||
        (TILT_EMERGENCY_MILLIS > 0 && tilt_emergency_started > 0 && (millis() - tilt_emergency_started) >= TILT_EMERGENCY_MILLIS)) {
        emergency_state |= (emergency_read & LL_EMERGENCY_BITS_LIFT);
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
    if (status_message.v_battery >= (BATT_EMPTY + 2.0f))
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
        switch (last_high_level_state.current_mode & 0b111111) {
            case HighLevelMode::MODE_IDLE:
                setLed(leds_message, LED_S1, LED_on);
                break;
            case HighLevelMode::MODE_AUTONOMOUS:
                setLed(leds_message, LED_S1, LED_blink_slow);
                break;
            default:
                setLed(leds_message, LED_S1, LED_blink_fast);
                break;
        }
        switch ((last_high_level_state.current_mode >> 6) & 0b11) {
            case 1:
                setLed(leds_message, LED_S2, LED_blink_slow);
                break;
            case 2:
                setLed(leds_message, LED_S2, LED_blink_fast);
                break;
            case 3:
                setLed(leds_message, LED_S2, LED_on);
                break;
            default:
                setLed(leds_message, LED_S2, LED_off);
                break;
        }
    }

    // Show Info mower lifted or stop button pressed
    if (status_message.emergency_bitmask & LL_EMERGENCY_BITS_STOP) {
        setLed(leds_message, LED_MOWER_LIFTED, LED_blink_fast);
    } else if (status_message.emergency_bitmask & LL_EMERGENCY_BITS_LIFT) {
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
    // Core
    digitalWrite(LED_BUILTIN, HIGH);
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
                    status_message.status_bitmask |= 0b00010000;
                } else {
                    status_message.status_bitmask &= 0b11101111;
                }
                mutex_exit(&mtx_status_message);

                break;
            case 6:
                mutex_enter_blocking(&mtx_status_message);
                if (state) {
                    status_message.status_bitmask |= 0b01000000;
                } else {
                    status_message.status_bitmask &= 0b10111111;
                }
                mutex_exit(&mtx_status_message);
                break;
            default:
                break;
        }
    }

    delay(100);
}

void setup() {
    //  We do hardware init in this core, so that we don't get invalid states.
    //  Therefore, we pause the other core until setup() was a success
    rp2040.idleOtherCore();

#ifdef USB_DEBUG
    DEBUG_SERIAL.begin(9600);
#endif

    emergency_latch = true;
    ROS_running = false;

    lift_emergency_started = 0;
    button_emergency_started = 0;
    // Initialize messages
    imu_message = {0};
    status_message = {0};
    imu_message.type = PACKET_ID_LL_IMU;
    status_message.type = PACKET_ID_LL_STATUS;

    // Setup pins
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_ENABLE_CHARGE, OUTPUT);
    digitalWrite(PIN_ENABLE_CHARGE, HIGH);

    gpio_init(PIN_RASPI_POWER);
    gpio_put(PIN_RASPI_POWER, true);
    gpio_set_dir(PIN_RASPI_POWER, true);
    gpio_put(PIN_RASPI_POWER, true);

    // Enable raspi power
    p.neoPixelSetValue(0, 32, 0, 0, true);
    delay(1000);
    setRaspiPower(true);
    p.neoPixelSetValue(0, 255, 0, 0, true);

    pinMode(PIN_MUX_OUT, OUTPUT);
    pinMode(PIN_MUX_ADDRESS_0, OUTPUT);
    pinMode(PIN_MUX_ADDRESS_1, OUTPUT);
    pinMode(PIN_MUX_ADDRESS_2, OUTPUT);

    pinMode(PIN_EMERGENCY_1, INPUT);
    pinMode(PIN_EMERGENCY_2, INPUT);
    pinMode(PIN_EMERGENCY_3, INPUT);
    pinMode(PIN_EMERGENCY_4, INPUT);

    analogReadResolution(12);

    // init serial com to RasPi
    PACKET_SERIAL.begin(115200);
    packetSerial.setStream(&PACKET_SERIAL);
    packetSerial.setPacketHandler(&onPacketReceived);

    UI1_SERIAL.begin(115200);
    UISerial.setStream(&UI1_SERIAL);
    UISerial.setPacketHandler(&onUIPacketReceived);

    /*
     * IMU INITIALIZATION
     */

    bool init_imu_success = false;
    int init_imu_tries = 1000;
    while(init_imu_tries --> 0) {
        if(init_imu()) {
            init_imu_success = true;
            break;
        }
#ifdef USB_DEBUG
        DEBUG_SERIAL.println("IMU initialization unsuccessful, retrying in 1 sec");
#endif
        p.neoPixelSetValue(0, 0, 0, 0, true);
        delay(1000);
        p.neoPixelSetValue(255, 255, 0, 0, true);
        delay(100);
        p.neoPixelSetValue(0, 0, 0, 0, true);
        delay(100);
    }

    if (!init_imu_success) {
#ifdef USB_DEBUG
        DEBUG_SERIAL.println("IMU initialization unsuccessful");
        DEBUG_SERIAL.println("Check IMU wiring or try cycling power");
#endif
        status_message.status_bitmask = 0;
        while (1) { // Blink RED for IMU failure
            p.neoPixelSetValue(0, 255, 0, 0, true);
            delay(500);
            p.neoPixelSetValue(0, 0, 0, 0, true);
            delay(500);
        }
    }
    p.neoPixelSetValue(0, 255, 255, 255, true);     // White for IMU Success

#ifdef USB_DEBUG
    DEBUG_SERIAL.println("Imu initialized");
#endif

    status_message.status_bitmask |= 1;

#ifdef ENABLE_SOUND_MODULE
    p.neoPixelSetValue(0, 0, 255, 255, true);

    sound_available = my_sound.begin();
    if (sound_available) {
        p.neoPixelSetValue(0, 0, 0, 255, true);
        my_sound.setvolume(100);
        my_sound.playSoundAdHoc(1);
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

    rp2040.resumeOtherCore();

    // Cover UI board clear all LEDs
    leds_message.type = Set_LEDs;
    leds_message.leds = 0;
    sendUIMessage(&leds_message, sizeof(leds_message));

    p.neoPixelSetValue(0, 255, 255, 255, true);     // White 1s final success
    delay(1000);
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
    }
    else if (buffer[0] == Get_Emergency && size == sizeof(struct msg_event_emergency))
    {
        struct msg_event_emergency *msg = (struct msg_event_emergency *)buffer;
        stock_ui_emergency_state = msg->state;
    }
    else if (buffer[0] == Get_Rain && size == sizeof(struct msg_event_rain))
    {
        struct msg_event_rain *msg = (struct msg_event_rain *)buffer;
        stock_ui_rain = (msg->value < msg->threshold);
    }
    else if (buffer[0] == Get_Subscribe && size == sizeof(struct msg_event_subscribe))
    {
        struct msg_event_subscribe *msg = (struct msg_event_subscribe *)buffer;
        ui_topic_bitmask = msg->topic_bitmask;
        ui_interval = msg->interval;
    }
}

void sendConfigMessage(uint8_t pkt_type) {
    struct ll_high_level_config ll_config;
    ll_config.type = pkt_type;
    ll_config.config_bitmask = config_bitmask;
    ll_config.volume = 80;            // FIXME: Adapt once nv_config or improve-sound got merged
    strncpy(ll_config.language, "en", 2); // FIXME: Adapt once nv_config or improve-sound got merged
    sendMessage(&ll_config, sizeof(struct ll_high_level_config));
}

void onPacketReceived(const uint8_t *buffer, size_t size) {
    // sanity check for CRC to work (1 type, 1 data, 2 CRC)
    if (size < 4)
        return;

    // check the CRC
    uint16_t crc;

    // @ClemensElflein: Here is why I decided against having CRC at first packet member:
    //   We need to select between the two CRC positions. But if CRC would be on first place, type would go somewhere behind it.
    //   But then type for this packet would be on a position where the other packets place their data(or CRC). This would
    //   quickly result in wrong packet-type identifications.
    //   Could also be solved, but would complicate code and double CRC calculations for the packets which got identified wrong.
    //   The alternative I do see could be a cascaded CRC calculation:
    //      Nearly 99.99% of all packets have their packet at the end, so we always could try that first, but when it fails,
    //      we could fall back and try with CRC on first position to see if it's this one-time packet.
    //      But I would prefer this more strict and defined way.

    // We have two CRC position variants
    if (buffer[0] == PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ || buffer[0] == PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP) {
        // Flexible length packet, where the CRC follows the type and type is NOT part of the CRC.

        // Comment: This is just as safe as including the type into the CRC because: If type would be accidentally wrong,
        // then it also wouldn't validate in the "else" CRC calculation where type is included
        crc = CRC16.ccitt(buffer + 3, size - 3);

        if (buffer[1] != ((crc >> 8) & 0xFF) ||
            buffer[2] != (crc & 0xFF))
            return;
    } else {
        // Normal, fixed length packet, where the CRC is the last member
        crc = CRC16.ccitt(buffer, size - 2);

        if (buffer[size - 1] != ((crc >> 8) & 0xFF) ||
            buffer[size - 2] != (crc & 0xFF))
            return;
    }

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

            // Send current LL config (and request HL config response)
            sendConfigMessage(PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ);
        }
    } else if (buffer[0] == PACKET_ID_LL_HIGH_LEVEL_STATE && size == sizeof(struct ll_high_level_state)) {
        // copy the state
        last_high_level_state = *((struct ll_high_level_state *) buffer);
    } else if ((buffer[0] == PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ || buffer[0] == PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP) &&
               (size == sizeof(struct ll_high_level_config) || size == sizeof(struct ll_high_level_config) + sizeof(struct ll_high_level_config_ext_v2))) {
        // Read and handle received config
        struct ll_high_level_config *pkt = (struct ll_high_level_config *)buffer;
        // Lower comms_version is leading
        pkt->comms_version <= LL_HIGH_LEVEL_CONFIG_MAX_COMMS_VERSION ? comms_version = pkt->comms_version : comms_version = LL_HIGH_LEVEL_CONFIG_MAX_COMMS_VERSION;
        config_bitmask = pkt->config_bitmask;  // Take over as sent. HL is leading (for now)

        // PR-80: Assign volume & language if not already stored in flash-config

        // Handle possible ll_high_level_config_ext_v2 extension
        if (comms_version >= 2) {
            // Read and handle extension v2
            struct ll_high_level_config_ext_v2 *pkt = (struct ll_high_level_config_ext_v2 *)buffer + sizeof(struct ll_high_level_config);

            // Fix exceed of absolute max. values
            pkt->v_charge_max = min(pkt->v_charge_max, V_CHARGE_ABS_MAX);
            pkt->i_charge_max = min(pkt->i_charge_max, I_CHARGE_ABS_MAX);

            // Copy extension packet to config_v2
            config_v2 = *((struct ll_high_level_config_ext_v2 *)buffer + sizeof(struct ll_high_level_config));

            // TODO: Parse hall config fields and build a compact iterable vector/array/... of active hall inputs or do it directly in updateEmergency()

            // TODO: Decide which fields should get stored in nv_config (flash) so that it's directly avail after power-up (without the need of ros_running)
            // My current thoughts are:
            //   v_charge_max for those who need a much lower or higher charging voltage, so that battery doesn't get empty or overload if ROS is offline for a longer period (or silently crashed)
            //   i_charge is similar. If one has a higher current DCDC and dock his empty mower, it should charge, even if ROS is down
            //   v_battery_max for sure (overcharge protection)
            //
            // but NOT:
            //   halls_config/inverted? Think we should always start with default OM halls config but if comms_version >= 2, wait till v2 packet extension got received.
            //   tilt/lift periods
        }

        if (buffer[0] == PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ)
            sendConfigMessage(PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP);
    }
}

// returns true, if it's a good idea to charge the battery (current, voltages, ...)
bool checkShouldCharge() {
    return status_message.v_charge < config_v2.v_charge_max && status_message.charging_current < config_v2.i_charge_max && status_message.v_battery < config_v2.v_battery_max;
}

void updateChargingEnabled() {
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
    }

    if (now - last_status_update_millis > STATUS_CYCLETIME) {
        updateNeopixel();

        status_message.v_battery =
                (float) analogRead(PIN_ANALOG_BATTERY_VOLTAGE) * (3.3f / 4096.0f) * ((VIN_R1 + VIN_R2) / VIN_R2);
        status_message.v_charge =
                (float) analogRead(PIN_ANALOG_CHARGE_VOLTAGE) * (3.3f / 4096.0f) * ((VIN_R1 + VIN_R2) / VIN_R2);
#ifndef IGNORE_CHARGING_CURRENT
        status_message.charging_current =
                (float) analogRead(PIN_ANALOG_CHARGE_CURRENT) * (3.3f / 4096.0f) / (CURRENT_SENSE_GAIN * R_SHUNT);
#else
        status_message.charging_current = -1.0f;
#endif
        status_message.status_bitmask = (status_message.status_bitmask & 0b11111011) | ((charging_allowed & 0b1) << 2);
        status_message.status_bitmask = (status_message.status_bitmask & 0b11011111) | ((sound_available & 0b1) << 5);

        // calculate percent value accu filling
        float delta = BATT_FULL - BATT_EMPTY;
        float vo = status_message.v_battery - BATT_EMPTY;
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
#ifdef ENABLE_SOUND_MODULE
        if (sound_available) {
            my_sound.processSounds();
        }
#endif
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

    packetSerial.send((uint8_t *) message, size);
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

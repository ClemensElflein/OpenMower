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
#include <MPU9250.h>
#include <FastCRC.h>
#include <PacketSerial.h>
#include <DFPlayerMini_Fast.h>
#include "datatypes.h"
#include "pins.h"
#include "ui_datatypes.h"

#define IMU_CYCLETIME 20          // cycletime for refresh IMU data
#define STATUS_CYCLETIME 100      // cycletime for refresh analog and digital Statusvalues
#define UI_SET_LED_CYCLETIME 1000 // cycletime for refresh UI status LEDs

#define LIFT_EMERGENCY_MILLIS 1500  // Time for wheels to be lifted in order to count as emergency. This is to filter uneven ground.
#define BUTTON_EMERGENCY_MILLIS 150 // Time for button emergency to activate. This is to debounce the button if triggered on bumpy surfaces

// Define to stream debugging messages via USB
// #define USB_DEBUG
// #define DEBUG_IMU

// Only define DEBUG_SERIAL if USB_DEBUG is actually enabled.
// This enforces compile errors if it's used incorrectly.
#ifdef USB_DEBUG
#define DEBUG_SERIAL Serial
#endif
#define PACKET_SERIAL Serial1
#define UI1_SERIAL Serial2

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

PacketSerial packetSerial; // COBS communication PICO <> Raspi
PacketSerial UISerial;     // COBS communication PICO UI-Board
FastCRC16 CRC16;

// MbedSPI IMU_SPI(16, 19, 18);
MPU9250 IMU(SPI, PIN_IMU_CS);
size_t fifoSize;

unsigned long last_imu_millis = 0;
unsigned long last_status_update_millis = 0;
unsigned long last_heartbeat_millis = 0;
unsigned long last_UILED_millis = 0;

unsigned long lift_emergency_started = 0;
unsigned long button_emergency_started = 0;

// Predefined message buffers, so that we don't need to allocate new ones later.
struct ll_imu imu_message = {0};
struct ll_status status_message = {0};
struct ui_command uiCommandStruct = {0};
// A mutex which is used by core1 each time status_message is modified.
// We can lock it during message transmission to prevent core1 to modify data in this time.
auto_init_mutex(mtx_status_message);

bool emergency_latch = true;

bool charging_allowed = false;
unsigned long charging_disabled_time = 0;

bool sound_available = false;
SerialPIO soundSerial(PIN_SOUND_TX, PIN_SOUND_RX);
DFPlayerMini_Fast myMP3;

void sendMessage(void *message, size_t size);
void sendUIMessage(void *message, size_t size);
void onPacketReceived(const uint8_t *buffer, size_t size);
void onUIPacketReceived(const uint8_t *buffer, size_t size);

void setRaspiPower(bool power)
{
  // Update status bits in the status message
  status_message.status_bitmask = (status_message.status_bitmask & 0b11111101) | ((power & 0b1) << 1);
  digitalWrite(PIN_RASPI_POWER, power);
}

void updateEmergency()
{

  if (millis() - last_heartbeat_millis > HEARTBEAT_MILLIS)
  {
    emergency_latch = true;
  }
  uint8_t last_emergency = status_message.emergency_bitmask & 1;

  // Mask the emergency bits. 2x Lift sensor, 2x Emergency Button
  uint8_t pin_states = gpio_get_all() & (0b11001100);
  uint8_t emergency_state = 0;

  bool is_lifted = ~pin_states & 0b00001100;
  bool stop_pressed = ~pin_states & 0b11000000;

  if (is_lifted && lift_emergency_started == 0)
  {
    // We just lifted, store the timestamp
    lift_emergency_started = millis();
  }
  else
  {
    // Not lifted, reset the time
    lift_emergency_started = 0;
  }

  if (stop_pressed && button_emergency_started == 0)
  {
    // We just pressed, store the timestamp
    button_emergency_started = millis();
  }
  else
  {
    // Not pressed, reset the time
    button_emergency_started = 0;
  }

  if (lift_emergency_started > 0 && millis() - lift_emergency_started >= LIFT_EMERGENCY_MILLIS)
  {
    // Emergency bit 2 (lift wheel 1)set?
    if (~pin_states & 0b00000100)
      emergency_state |= 0b01000;
    // Emergency bit 1 (lift wheel 2)set?
    if (~pin_states & 0b00001000)
      emergency_state |= 0b10000;
  }

  if (button_emergency_started > 0 && millis() - button_emergency_started >= BUTTON_EMERGENCY_MILLIS)
  {
    // Emergency bit 2 (stop button) set?
    if (~pin_states & 0b01000000)

      emergency_state |= 0b00010;
    // Emergency bit 1 (stop button)set?
    if (~pin_states & 0b10000000)
      emergency_state |= 0b00100;
  }

  if (emergency_state || emergency_latch)
  {
    emergency_latch |= 1;
    emergency_state |= 1;
  }

  status_message.emergency_bitmask = emergency_state;

  // If it's a new emergency, instantly send the message. This is to not spam the channel during emergencies.
  if (last_emergency != (emergency_state & 1))
  {
    sendMessage(&status_message, sizeof(struct ll_status));

    // Show Info mower lifted or stop button pressed
    if (status_message.emergency_bitmask & 0b00110)
      uiCommandStruct.cmd2 = LED_blink_fast;
    else if (status_message.emergency_bitmask & 0b11000)
      uiCommandStruct.cmd2 = LED_blink_slow;
    else if (status_message.emergency_bitmask)
      // On for other emergencies
      uiCommandStruct.cmd2 = LED_on;
    else
      uiCommandStruct.cmd2 = LED_off;
    uiCommandStruct.type = Set_LED;
    uiCommandStruct.cmd1 = MOWER_LIFTED;
    sendUIMessage(&uiCommandStruct, sizeof(struct ui_command));
  }
}

// deals with the pyhsical information an control the UI-LEDs und buzzer in depency of voltage und current values
void manageUILEDS()
{
  struct ui_command uiCommandStruct = {0};

  // Schow Info Docking LED
  if ((status_message.charging_current > 0.80f) && (status_message.v_charge > 20.0f))
    uiCommandStruct.cmd2 = LED_blink_fast;
  else if ((status_message.charging_current <= 0.80f) && (status_message.charging_current >= 0.15f) && (status_message.v_charge > 20.0f))
    uiCommandStruct.cmd2 = LED_blink_slow;
  else if ((status_message.charging_current < 0.15f) && (status_message.v_charge > 20.0f))
    uiCommandStruct.cmd2 = LED_on;
  else
    uiCommandStruct.cmd2 = LED_off;
  uiCommandStruct.type = Set_LED;
  uiCommandStruct.cmd1 = CHARGING;
  sendUIMessage(&uiCommandStruct, sizeof(struct ui_command));

  // Show Info Battery state
  if (status_message.v_battery >= (BATT_FULL - 0.5f))
    uiCommandStruct.cmd2 = LED_on;
  else if (status_message.v_battery <= (BATT_EMPTY + 1.5f))
    uiCommandStruct.cmd2 = LED_blink_fast;
  else
    uiCommandStruct.cmd2 = LED_blink_slow;
  uiCommandStruct.type = Set_LED;
  uiCommandStruct.cmd1 = BATTERY_LOW;
  sendUIMessage(&uiCommandStruct, sizeof(struct ui_command));

  // Show percent value akkupower but only, if mower is not docked
  uiCommandStruct.type = Set_LED;
  uiCommandStruct.cmd1 = LED_BAR;
  if (status_message.v_charge < 10.0f) // activate only when undocked
  {
    // use the second LED row as bargraph

    uiCommandStruct.cmd2 = LED_on;
    uiCommandStruct.cmd3 = status_message.batt_percentage;
    sendUIMessage(&uiCommandStruct, sizeof(struct ui_command));
  }
  else
  {
    uiCommandStruct.cmd2 = LED_off;
    uiCommandStruct.cmd3 = 0;
    sendUIMessage(&uiCommandStruct, sizeof(struct ui_command));
  }

  // Show Info mower lifted or stop button pressed
  if (status_message.emergency_bitmask & 0b00110)
    uiCommandStruct.cmd2 = LED_blink_fast;
  else if (status_message.emergency_bitmask & 0b11000)
    uiCommandStruct.cmd2 = LED_blink_slow;
  else if (status_message.emergency_bitmask)
    // On for other emergencies
    uiCommandStruct.cmd2 = LED_on;
  else
    uiCommandStruct.cmd2 = LED_off;
  uiCommandStruct.type = Set_LED;
  uiCommandStruct.cmd1 = MOWER_LIFTED;
  sendUIMessage(&uiCommandStruct, sizeof(struct ui_command));
}

void setup1()
{
  // Core
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop1()
{
  // Loop through the mux and query actions. Store the result in the multicore fifo
  for (uint8_t mux_address = 0; mux_address < 7; mux_address++)
  {
    gpio_put_masked(0b111 << 13, mux_address << 13);
    delay(1);
    bool state = gpio_get(PIN_MUX_IN);

    switch (mux_address)
    {
    case 5:
      mutex_enter_blocking(&mtx_status_message);

      if (state)
      {
        status_message.status_bitmask |= 0b00010000;
      }
      else
      {
        status_message.status_bitmask &= 0b11101111;
      }
      mutex_exit(&mtx_status_message);

      break;
    case 6:
      mutex_enter_blocking(&mtx_status_message);
      if (state)
      {
        status_message.status_bitmask |= 0b00100000;
      }
      else
      {
        status_message.status_bitmask &= 0b11011111;
      }
      mutex_exit(&mtx_status_message);
      break;
    default:
      break;
    }
  }

  delay(100);
}

void setup()
{
  p.neoPixelSetValue(0, 255, 0, 0, true);
  // We do hardware init in this core, so that we don't get invalid states.
  // Therefore, we pause the other core until setup() was a success
  rp2040.idleOtherCore();

  emergency_latch = true;
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
  digitalWrite(PIN_ENABLE_CHARGE, charging_allowed);

  gpio_init(PIN_RASPI_POWER);
  gpio_put(PIN_RASPI_POWER, true);
  gpio_set_dir(PIN_RASPI_POWER, true);
  gpio_put(PIN_RASPI_POWER, true);

  // Enable raspi power
  setRaspiPower(true);

  pinMode(PIN_MUX_OUT, OUTPUT);
  pinMode(PIN_MUX_ADDRESS_0, OUTPUT);
  pinMode(PIN_MUX_ADDRESS_1, OUTPUT);
  pinMode(PIN_MUX_ADDRESS_2, OUTPUT);

  pinMode(PIN_EMERGENCY_1, INPUT);
  pinMode(PIN_EMERGENCY_2, INPUT);
  pinMode(PIN_EMERGENCY_3, INPUT);
  pinMode(PIN_EMERGENCY_4, INPUT);

  analogReadResolution(12);

#ifdef USB_DEBUG
  DEBUG_SERIAL.begin(115200);
#endif

  PACKET_SERIAL.begin(115200);
  packetSerial.setStream(&PACKET_SERIAL);
  packetSerial.setPacketHandler(&onPacketReceived);

  UI1_SERIAL.setRX(5); // set hardware pin
  UI1_SERIAL.setTX(4);
  UI1_SERIAL.begin(115200);
  UISerial.setStream(&UI1_SERIAL);
  UISerial.setPacketHandler(&onUIPacketReceived);

  /*
   * IMU INITIALIZATION
   */

  int status = IMU.begin();
  if (status < 0)
  {
#ifdef USB_DEBUG
    DEBUG_SERIAL.println("IMU initialization unsuccessful");
    DEBUG_SERIAL.println("Check IMU wiring or try cycling power");
    DEBUG_SERIAL.print("Status: ");
    DEBUG_SERIAL.println(status);
#endif
    status_message.status_bitmask = 0;
    while (1)
    {
#ifdef USB_DEBUG
      DEBUG_SERIAL.println("Error: Imu init failed");
#endif
      // We don't need to lock the mutex here, since core 1 is sleeping anyways
      sendMessage(&status_message, sizeof(struct ll_status));
      delay(1000);
    }
  }

  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);

#ifdef USB_DEBUG
  DEBUG_SERIAL.println("Imu initialized");
#endif

  /*
   * /IMU INITIALIZATION
   */

  status_message.status_bitmask |= 1;

  // sound init
  soundSerial.begin(9600);
  while (soundSerial.available())
    soundSerial.read();
  sound_available = myMP3.begin(soundSerial);

  if (sound_available)
  {
    p.neoPixelSetValue(0, 0, 0, 255, true);
    p.neoPixelShow();
    myMP3.volume(30);
    myMP3.play(1);
  }
  else
  {
    for (uint8_t b = 0; b < 3; b++)
    {
      p.neoPixelSetValue(0, 0, 0, 0, true);
      p.neoPixelShow();
      delay(100);
      p.neoPixelSetValue(0, 0, 0, 255, true);
      p.neoPixelShow();
      delay(100);
    }
  }

  rp2040.resumeOtherCore();

  // UIboard clear all LEDs
  uiCommandStruct.type = Set_LED;
  uiCommandStruct.cmd1 = 0;
  uiCommandStruct.cmd2 = LED_All_OFF;
  sendUIMessage(&uiCommandStruct, sizeof(struct ui_command));
}

void onUIPacketReceived(const uint8_t *buffer, size_t size)
{

  u_int16_t *crc_pointer = (uint16_t *)(buffer + (size - 2));
  u_int16_t readcrc = *crc_pointer;

  // check structure size
  if (size != sizeof(struct ui_command))
    return;
  if ((buffer[0] != Get_Version) && (buffer[0] != Get_Buttonnr))
    return;

  // check the CRC
  uint16_t crc = CRC16.ccitt(buffer, size - 2);

  if (buffer[size - 1] != ((crc >> 8) & 0xFF) ||
      buffer[size - 2] != (crc & 0xFF))
    return;

  struct ui_command *buttonboard = (struct ui_command *)buffer;
  // overwrite type for the ROS system
  buttonboard->type = PACKET_ID_LL_UI_EVENT;
  sendMessage(buttonboard, sizeof(struct ui_command));
}

void onPacketReceived(const uint8_t *buffer, size_t size)
{
  // we currently only support heartbeats
  if (size != sizeof(struct ll_heartbeat))
    return;
  if (buffer[0] != PACKET_ID_LL_HEARTBEAT)
    return;

  // check the CRC
  uint16_t crc = CRC16.ccitt(buffer, size - 2);

  if (buffer[size - 1] != ((crc >> 8) & 0xFF) ||
      buffer[size - 2] != (crc & 0xFF))
    return;

  // CRC and packet is OK, reset watchdog
  last_heartbeat_millis = millis();
  struct ll_heartbeat *heartbeat = (struct ll_heartbeat *)buffer;
  if (heartbeat->emergency_release_requested)
  {
    emergency_latch = false;
  }
  // Check in this order, so we can set it again in the same packet if required.
  if (heartbeat->emergency_requested)
  {
    emergency_latch = true;
  }
}

// returns true, if it's a good idea to charge the battery (current, voltages, ...)
bool checkShouldCharge()
{
  return status_message.v_charge < 30.0 && status_message.charging_current < 1.5 && status_message.v_battery < 29.0;
}

void updateChargingEnabled()
{
  if (charging_allowed)
  {
    if (!checkShouldCharge())
    {
      digitalWrite(PIN_ENABLE_CHARGE, false);
      charging_allowed = false;
      charging_disabled_time = millis();
    }
  }
  else
  {
    // enable charging after CHARGING_RETRY_MILLIS
    if (millis() - charging_disabled_time > CHARGING_RETRY_MILLIS)
    {
      if (!checkShouldCharge())
      {
        digitalWrite(PIN_ENABLE_CHARGE, false);
        charging_allowed = false;
        charging_disabled_time = millis();
      }
      else
      {
        digitalWrite(PIN_ENABLE_CHARGE, true);
        charging_allowed = true;
      }
    }
  }
}

void loop()
{
  packetSerial.update();
  UISerial.update();

  updateChargingEnabled();
  updateEmergency();

  unsigned long now = millis();
  if (now - last_imu_millis > IMU_CYCLETIME)
  {
    IMU.readSensor();
    imu_message.type = PACKET_ID_LL_IMU;
    imu_message.acceleration_mss[0] = IMU.getAccelX_mss();
    imu_message.acceleration_mss[1] = IMU.getAccelY_mss();
    imu_message.acceleration_mss[2] = IMU.getAccelZ_mss();

    imu_message.gyro_rads[0] = IMU.getGyroX_rads();
    imu_message.gyro_rads[1] = IMU.getGyroY_rads();
    imu_message.gyro_rads[2] = IMU.getGyroZ_rads();

    imu_message.mag_uT[0] = IMU.getMagX_uT();
    imu_message.mag_uT[1] = IMU.getMagY_uT();
    imu_message.mag_uT[2] = IMU.getMagZ_uT();

    imu_message.dt_millis = now - last_imu_millis;
    sendMessage(&imu_message, sizeof(struct ll_imu));

    last_imu_millis = now;

#ifdef USB_DEBUG
#ifdef DEBUG_IMU
    DEBUG_SERIAL.print(IMU.getAccelX_mss(), 6);
    DEBUG_SERIAL.print("\t");
    DEBUG_SERIAL.print(IMU.getAccelY_mss(), 6);
    DEBUG_SERIAL.print("\t");
    DEBUG_SERIAL.print(IMU.getAccelZ_mss(), 6);
    DEBUG_SERIAL.print("\t");
    DEBUG_SERIAL.print(IMU.getGyroX_rads(), 6);
    DEBUG_SERIAL.print("\t");
    DEBUG_SERIAL.print(IMU.getGyroY_rads(), 6);
    DEBUG_SERIAL.print("\t");
    DEBUG_SERIAL.print(IMU.getGyroZ_rads(), 6);
    DEBUG_SERIAL.print("\t");
    DEBUG_SERIAL.print(IMU.getMagX_uT(), 6);
    DEBUG_SERIAL.print("\t");
    DEBUG_SERIAL.print(IMU.getMagY_uT(), 6);
    DEBUG_SERIAL.print("\t");
    DEBUG_SERIAL.print(IMU.getMagZ_uT(), 6);
    DEBUG_SERIAL.print("\t");
    DEBUG_SERIAL.println(IMU.getTemperature_C(), 6);
#endif
#endif
  }

  if (now - last_status_update_millis > STATUS_CYCLETIME)
  {

    status_message.v_battery = (float)analogRead(PIN_ANALOG_BATTERY_VOLTAGE) * (3.3f / 4096.0f) * ((VIN_R1 + VIN_R2) / VIN_R2);
    status_message.v_charge = (float)analogRead(PIN_ANALOG_CHARGE_VOLTAGE) * (3.3f / 4096.0f) * ((VIN_R1 + VIN_R2) / VIN_R2);
    status_message.charging_current = (float)analogRead(PIN_ANALOG_CHARGE_CURRENT) * (3.3f / 4096.0f) / (CURRENT_SENSE_GAIN * R_SHUNT);
    status_message.status_bitmask = (status_message.status_bitmask & 0b11111011) | ((charging_allowed & 0b1) << 2);
    status_message.status_bitmask = (status_message.status_bitmask & 0b11011111) | ((sound_available & 0b1) << 5);

    // calculate percent value accu filling
    float delta = BATT_FULL - BATT_EMPTY;
    float vo = status_message.v_battery - BATT_EMPTY;
    status_message.batt_percentage = vo / delta * 100;

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

  if (now - last_UILED_millis > UI_SET_LED_CYCLETIME)
  {

    manageUILEDS();
    last_UILED_millis = now;
  }
}

void sendMessage(void *message, size_t size)
{
  // packages need to be at least 1 byte of type, 1 byte of data and 2 bytes of CRC
  if (size < 4)
  {
    return;
  }
  uint8_t *data_pointer = (uint8_t *)message;

  // calculate the CRC
  uint16_t crc = CRC16.ccitt((uint8_t *)message, size - 2);
  data_pointer[size - 1] = (crc >> 8) & 0xFF;
  data_pointer[size - 2] = crc & 0xFF;

  packetSerial.send((uint8_t *)message, size);
}

void sendUIMessage(void *message, size_t size)
{
  // packages need to be at least 1 byte of type, 1 byte of data and 2 bytes of CRC
  if (size < 4)
  {
    return;
  }
  uint8_t *data_pointer = (uint8_t *)message;

  // calculate the CRC
  uint16_t crc = CRC16.ccitt((uint8_t *)message, size - 2);
  data_pointer[size - 1] = (crc >> 8) & 0xFF;
  data_pointer[size - 2] = crc & 0xFF;

  UISerial.send((uint8_t *)message, size);
}

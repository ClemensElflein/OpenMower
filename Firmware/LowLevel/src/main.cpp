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
#include <MPU9250.h>
#include <FastCRC.h>
#include <PacketSerial.h>

#include "datatypes.h"
#include "pins.h"

// Define to stream debugging messages via USB
// #define USB_DEBUG
// #define DEBUG_IMU
#define DEBUG_SERIAL Serial
#define PACKET_SERIAL Serial1

/**
 * @brief Some hardware parameters
 */
#define VIN_R1 10000.0f
#define VIN_R2 1000.0f
#define R_SHUNT 0.033f
#define CURRENT_SENSE_GAIN 100.0f

// Emergency will be engaged, if no heartbeat was received in this time frame.
#define HEARTBEAT_MILLIS 500

PacketSerial packetSerial;
FastCRC16 CRC16;

// MbedSPI IMU_SPI(16, 19, 18);
MPU9250 IMU(SPI, PIN_IMU_CS);
size_t fifoSize;
unsigned long last_imu_millis = 0;
unsigned long last_status_update_millis = 0;
unsigned long last_heartbeat_millis = 0;

// Predefined message buffers, so that we don't need to allocate new ones later.
struct ll_imu imu_message = {0};
struct ll_status status_message = {0};
// A mutex which is used by core1 each time status_message is modified.
// We can lock it during message transmission to prevent core1 to modify data in this time.
auto_init_mutex(mtx_status_message);

bool emergency_latch = true;


void sendMessage(void *message, size_t size);
void onPacketReceived(const uint8_t *buffer, size_t size);

void setPower(uint8_t power)
{
  // Update status bits in the status message
  status_message.status_bitmask = (status_message.status_bitmask & 0b11110001) | ((power & 0b111) << 1);
  digitalWrite(PIN_RASPI_POWER, power & 0b1);
  digitalWrite(PIN_GPS_POWER, power & 0b10);
  digitalWrite(PIN_ESC_POWER, power & 0b100);
}

void updateEmergency()
{
  if(millis() - last_heartbeat_millis > HEARTBEAT_MILLIS) {
    emergency_latch = true;
  }
  uint8_t current_emergency = status_message.emergency_bitmask & 1;
  uint8_t pin_states = gpio_get_all() & (0b11001100);
  uint8_t emergency_state = ((~pin_states >> 2) & 0b11) | ((~pin_states >> 4) & 0b1100);
  emergency_state <<= 1;
  if (emergency_state || emergency_latch)
  {
    emergency_latch |= 1;
    emergency_state |= 1;
  }
  status_message.emergency_bitmask = emergency_state;
  if (current_emergency != (emergency_state & 1))
  {
    sendMessage(&status_message, sizeof(struct ll_status));
  }
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
  // We do hardware init in this core, so that we don't get invalid states.
  // Therefore, we pause the other core until setup() was a success
  rp2040.idleOtherCore();

  emergency_latch = false;
  // Initialize messages
  imu_message = {0};
  status_message = {0};
  imu_message.type = PACKET_ID_LL_IMU;
  status_message.type = PACKET_ID_LL_STATUS;

  // Setup pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_GPS_POWER, OUTPUT);
  pinMode(PIN_ESC_POWER, OUTPUT);
  pinMode(PIN_RASPI_POWER, OUTPUT);
  pinMode(PIN_MUX_OUT, OUTPUT);
  pinMode(PIN_MUX_ADDRESS_0, OUTPUT);
  pinMode(PIN_MUX_ADDRESS_1, OUTPUT);
  pinMode(PIN_MUX_ADDRESS_2, OUTPUT);

  pinMode(PIN_EMERGENCY_1, INPUT);
  pinMode(PIN_EMERGENCY_2, INPUT);
  pinMode(PIN_EMERGENCY_3, INPUT);
  pinMode(PIN_EMERGENCY_4, INPUT);

  analogReadResolution(16);

  // Enable all power
  setPower(0b111);

#ifdef USB_DEBUG
  DEBUG_SERIAL.begin(115200);
#endif

  PACKET_SERIAL.begin(115200);
  packetSerial.setStream(&PACKET_SERIAL);
  packetSerial.setPacketHandler(&onPacketReceived);

  /*
   * IMU INITIALIZATION
   */

  int status = IMU.begin();
  if (status < 0)
  {
    DEBUG_SERIAL.println("IMU initialization unsuccessful");
    DEBUG_SERIAL.println("Check IMU wiring or try cycling power");
    DEBUG_SERIAL.print("Status: ");
    DEBUG_SERIAL.println(status);
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

  rp2040.resumeOtherCore();
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
  struct ll_heartbeat* heartbeat = (struct ll_heartbeat*)buffer;
  if(heartbeat->emergency_release_requested) {
    emergency_latch = false;
  }
  // Check in this order, so we can set it again in the same packet if required.
  if(heartbeat->emergency_requested) {
    emergency_latch = true;
  }
}

void loop()
{
  packetSerial.update();
  updateEmergency();

  unsigned long now = millis();
  if (now - last_imu_millis > 20)
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

  if (now - last_status_update_millis > 100)
  {
    status_message.v_system = (float)analogRead(PIN_ANALOG_VOLTAGE) * (3.3f / 65536.0f) * ((VIN_R1 + VIN_R2) / VIN_R2);
    status_message.v_charge = (float)analogRead(PIN_ANALOG_CHARGE_VOLTAGE) * (3.3f / 65536.0f) * ((VIN_R1 + VIN_R2) / VIN_R2);
    status_message.charging_current = (float)analogRead(PIN_ANALOG_CHARGE_CURRENT) * (3.3f / 65536.0f) / (CURRENT_SENSE_GAIN * R_SHUNT);

    mutex_enter_blocking(&mtx_status_message);
    sendMessage(&status_message, sizeof(struct ll_status));
    mutex_exit(&mtx_status_message);

    last_status_update_millis = now;
#ifdef USB_DEBUG
    DEBUG_SERIAL.print("status: 0b");
    DEBUG_SERIAL.print(status_message.status_bitmask, BIN);
    DEBUG_SERIAL.print("\t");

    DEBUG_SERIAL.print("vin: ");
    DEBUG_SERIAL.print(status_message.v_system, 3);
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
}

void sendMessage(void *message, size_t size)
{
  // packages need to be at least 1 byte of type, 1 byte of data and 2 bytes of CRC
  if (size < 4)
  {
    return;
  }
  uint8_t *data_pointer = (uint8_t *)message;
  uint16_t *crc_pointer = (uint16_t *)(data_pointer + (size - 3));
  // calculate the CRC
  uint16_t crc = CRC16.ccitt((uint8_t *)message, size - 2);
  data_pointer[size - 1] = (crc >> 8) & 0xFF;
  data_pointer[size - 2] = crc & 0xFF;

  packetSerial.send((uint8_t *)message, size);
}

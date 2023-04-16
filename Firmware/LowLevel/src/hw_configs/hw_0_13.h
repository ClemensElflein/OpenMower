#ifndef HW_0_13_H
#define HW_0_13_H

#include <SPI.h>

/**
 * This HW version uses the following Resources:
 * UART0 - ROS Communication
 * SPI0 / UART1 - IMU
 * PIO Neopixel
 * PIO UART: LED
 * PIO UART: Sound
 */

#define WT901_WIRE Wire

#define PIN_WT901_SDA 8
#define PIN_WT901_SCL 9

#define PIN_IMU_CS 5
#define PIN_IMU_MOSI 7
#define PIN_IMU_MISO 4
#define PIN_IMU_SCK 6

#define PIN_ANALOG_BATTERY_VOLTAGE 27
#define PIN_ANALOG_CHARGE_VOLTAGE 26
#define PIN_ANALOG_CHARGE_CURRENT 28

#define PIN_ENABLE_CHARGE 22

#define PIN_ESC_SHUTDOWN 20
#define PIN_RASPI_POWER 21

#define PIN_EMERGENCY_1 18
#define PIN_EMERGENCY_2 19
#define PIN_EMERGENCY_3 3
#define PIN_EMERGENCY_4 2

#define PIN_MUX_IN 11
#define PIN_MUX_OUT 12
#define PIN_MUX_ADDRESS_0 13
#define PIN_MUX_ADDRESS_1 14
#define PIN_MUX_ADDRESS_2 15

#define PIN_NEOPIXEL 10

#define PIN_UI_TX 8
#define PIN_UI_RX 9

#ifdef ENABLE_SOUND_MODULE
#define PIN_SOUND_TX 16
#define PIN_SOUND_RX 17
#endif

void hw_assign_pins() {
    // Assign the correct pins to the IMU's hardware SPI.
    SPI.setCS(PIN_IMU_CS);
    SPI.setSCK(PIN_IMU_SCK);
    SPI.setRX(PIN_IMU_MISO);
    SPI.setTX(PIN_IMU_MOSI);
}

#endif
#pragma once

#ifdef HW_0_9_X
#define PIN_IMU_CS 17
#define PIN_ANALOG_BATTERY_VOLTAGE 27
#define PIN_ANALOG_CHARGE_VOLTAGE 26
#define PIN_ANALOG_CHARGE_CURRENT 28

#define PIN_ENABLE_CHARGE 22

#define PIN_ESC_SHUTDOWN 20
#define PIN_RASPI_POWER 21

#define PIN_EMERGENCY_1 7
#define PIN_EMERGENCY_2 6
#define PIN_EMERGENCY_3 3
#define PIN_EMERGENCY_4 2

#define PIN_MUX_IN 11
#define PIN_MUX_OUT 12
#define PIN_MUX_ADDRESS_0 13
#define PIN_MUX_ADDRESS_1 14
#define PIN_MUX_ADDRESS_2 15

#define PIN_NEOPIXEL 10

#define PIN_UI_TX 4
#define PIN_UI_RX 5

#ifdef ENABLE_SOUND_MODULE
#define PIN_SOUND_TX 8
#define PIN_SOUND_RX 9
#endif

#ifdef WT901_INSTEAD_OF_SOUND
#ifdef ENABLE_SOUND_MODULE
#error you can not enable sound and have wt901 on sound port at the same time.
#endif
#define PIN_WT901_TX 8
#define PIN_WT901_RX 9
#elif WT901 //This is to use WT901 on MPU9250 Slot via Serial.
#define PIN_WT901_TX 17
#define PIN_WT901_RX 16
#endif

#elif HW_0_10_X || HW_0_11_X || HW_0_12_X
#define WT901_WIRE Wire

#define PIN_WT901_SDA 8
#define PIN_WT901_SCL 9

#define PIN_IMU_CS 9
#define PIN_IMU_MOSI 7
#define PIN_IMU_MISO 8
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

#define PIN_UI_TX 4
#define PIN_UI_RX 5

#ifdef ENABLE_SOUND_MODULE
#define PIN_SOUND_TX 16
#define PIN_SOUND_RX 17
#endif

#elif HW_0_13_X
#define PIN_IMU_SCK 6
#define PIN_IMU_TX 7
#define PIN_IMU_RX 4
#define PIN_IMU_CS 5

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

#else
#error No hardware version defined
#endif

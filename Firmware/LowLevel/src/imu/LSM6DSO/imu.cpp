#include "imu.h"
#include "pins.h"
#include <LSM6DSOSensor.h>

#if defined(HW_0_10_X) || defined(HW_0_11_X) || defined(HW_0_12_X)
// Needs software SPI because pins were messed up in 0.10-0.12
#include <SoftwareSPI.h>
SoftwareSPI spiBus(PIN_IMU_SCK, PIN_IMU_MISO, PIN_IMU_MOSI);
#else
#include <SPI.h>
#define spiBus SPI
#endif

LSM6DSOSensor IMU(&spiBus, PIN_IMU_CS, 1000000);
int32_t accelerometer[3];
int32_t gyroscope[3];

bool init_imu() {
#if defined(HW_0_10_X) || defined(HW_0_11_X) || defined(HW_0_12_X)
  spiBus.begin();
#else
  spiBus.setCS(PIN_IMU_CS);
  spiBus.setTX(PIN_IMU_TX);
  spiBus.setRX(PIN_IMU_RX);
  spiBus.setSCK(PIN_IMU_SCK);
  spiBus.begin();
#endif

    int status = IMU.begin();

    uint8_t WHOAMI = 0;
    IMU.ReadID(&WHOAMI);
    if(WHOAMI != 0b01101100)
        return false;

    if (status != 0)
        return false;

    if (IMU.Enable_G() != 0)
        return false;

    if (IMU.Enable_X() != 0)
        return false;
    return true;
}

bool imu_read(float *acceleration_mss, float *gyro_rads, float *mag_uT) {
    bool success = true;
    success &= IMU.Get_X_Axes(accelerometer) == 0;
    success &= IMU.Get_G_Axes(gyroscope) == 0;

    // Left down: Y = -g
    acceleration_mss[1] = accelerometer[0] * 9.81 / 1000.0;
    // Nose down: X = -g
    acceleration_mss[0] = -accelerometer[1] * 9.81 / 1000.0;
    // Flat: Z = +g
    acceleration_mss[2] = accelerometer[2] * 9.81 / 1000.0;

    // Datasheet shows gyro and acceleromter axes are aligned
    gyro_rads[1] = -gyroscope[0] * (PI / 180.0) / 1000.0;
    gyro_rads[0] = gyroscope[1] * (PI / 180.0) / 1000.0;
    gyro_rads[2] = gyroscope[2] * (PI / 180.0) / 1000.0;

    mag_uT[0] = 0;
    mag_uT[1] = 0;
    mag_uT[2] = 0;

    return success;
}

void imu_loop() {}

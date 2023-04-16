#include "imu.h"
#include "hw.h"
#include <LSM6DSOSensor.h>


LSM6DSOSensor IMU(&IMU_SPI, PIN_IMU_CS, 1000000);
int32_t accelerometer[3];
int32_t gyroscope[3];

bool init_imu()
{
    IMU_SPI.begin();
    int status = IMU.begin();
    if (status != 0)
        return false;

    if (IMU.Enable_G() != 0)
        return false;

    if (IMU.Enable_X() != 0)
        return false;
    return true;
}

bool imu_read(float *acceleration_mss, float *gyro_rads, float *mag_uT)
{
    bool success = true;
    success &= IMU.Get_X_Axes(accelerometer) == 0;
    success &= IMU.Get_G_Axes(gyroscope) == 0;

    acceleration_mss[0] = accelerometer[0] * 9.81 / 1000.0;
    acceleration_mss[1] = accelerometer[1] * 9.81 / 1000.0;
    acceleration_mss[2] = accelerometer[2] * 9.81 / 1000.0;

    gyro_rads[0] = gyroscope[0] * (PI/180.0) / 1000.0;
    gyro_rads[1] = gyroscope[1] * (PI/180.0) / 1000.0;
    gyro_rads[2] = gyroscope[2] * (PI/180.0) / 1000.0;

    mag_uT[0] = 0;
    mag_uT[1] = 0;
    mag_uT[2] = 0;

    return success;
}

void imu_loop()
{
}

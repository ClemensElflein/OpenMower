#include "imu.h"
#include "pins.h"
#include <JY901.h>

CJY901 IMU(&Serial2);

bool init_imu()
{
    Serial2.setRX(PIN_WT901_RX); // set hardware pin
    Serial2.setTX(PIN_WT901_TX);

    IMU.begin();

    return true;
}

bool imu_read(float *acceleration_mss, float *gyro_rads, float *mag_uT)
{
    acceleration_mss[0] = (float)IMU.stcAcc.a[0] / 32768.0f * 16.0f * 9.81f;
    acceleration_mss[1] = (float)IMU.stcAcc.a[1] / 32768.0f * 16.0f * 9.81f;
    acceleration_mss[2] = (float)IMU.stcAcc.a[2] / 32768.0f * 16.0f * 9.81f;

    gyro_rads[0] = (float)IMU.stcGyro.w[0]/32768.0f*2000.0f * PI / 180.0f;
    gyro_rads[1] = (float)IMU.stcGyro.w[1]/32768.0f*2000.0f * PI / 180.0f;
    gyro_rads[2] = (float)IMU.stcGyro.w[2]/32768.0f*2000.0f * PI / 180.0f;

    mag_uT[0] = IMU.stcMag.h[0] / 1000.0f;
    mag_uT[1] = IMU.stcMag.h[1] / 1000.0f;
    mag_uT[2] = IMU.stcMag.h[2] / 1000.0f;

    return true;
}

void imu_loop()
{
    IMU.update();
}
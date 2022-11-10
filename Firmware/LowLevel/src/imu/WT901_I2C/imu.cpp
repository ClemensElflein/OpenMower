#include "imu.h"
#include "pins.h"
#include "Wire.h"
#include <JY901.h>

CJY901 IMU = CJY901(&WT901_WIRE);

bool init_imu()
{
    WT901_WIRE.setSDA(PIN_WT901_SDA);
    WT901_WIRE.setSCL(PIN_WT901_SCL);
    

    IMU.StartIIC();

    return true;
}

bool imu_read(float *acceleration_mss, float *gyro_rads, float *mag_uT)
{
    IMU.GetTime();
    IMU.GetAcc();
    IMU.GetMag();
    IMU.GetGyro();

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

}
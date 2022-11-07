#include "imu.h"
#include "pins.h"
#include <MPU9250.h>

MPU9250 IMU(SPI, PIN_IMU_CS);

bool init_imu()
{
    int status = IMU.begin();
    if (status < 0)
        return false;
    
    // setting DLPF bandwidth to 20 Hz
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);

    return true;
}

bool imu_read(float *acceleration_mss, float *gyro_rads, float *mag_uT)
{
    IMU.readSensor();

    acceleration_mss[0] = IMU.getAccelX_mss();
    acceleration_mss[1] = IMU.getAccelY_mss();
    acceleration_mss[2] = IMU.getAccelZ_mss();

    gyro_rads[0] = IMU.getGyroX_rads();
    gyro_rads[1] = IMU.getGyroY_rads();
    gyro_rads[2] = -IMU.getGyroZ_rads();

    mag_uT[0] = IMU.getMagX_uT();
    mag_uT[1] = IMU.getMagY_uT();
    mag_uT[2] = IMU.getMagZ_uT();

    return true;
}

void imu_loop() {

}

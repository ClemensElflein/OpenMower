#ifndef _IMU_H
#define _IMU_H

bool init_imu();

bool imu_read(float *acceleration_mss,float *gyro_rads,float *mag_uT);

// call once per loop, used to process serial or do sensor fusion or do nothing.
void imu_loop();

#endif
#ifndef _IMU_H
#define _IMU_H

bool init_imu();

bool imu_read(float *acceleration_mss,float *gyro_rads,float *mag_uT);

// call once per loop, used to process serial or do sensor fusion or do nothing.
void imu_loop();

#if defined(WT901) || defined(WT901_INSTEAD_OF_SOUND)
bool imu_comms_error(); // get IMU communication error flag (and reset it)
#endif

#endif
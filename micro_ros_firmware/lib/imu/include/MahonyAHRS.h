#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H


void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax,
    float ay, float az);

#endif

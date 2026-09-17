#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

extern volatile float twoKp;
extern volatile float twoKi;

void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax,
                         float ay, float az);

#endif

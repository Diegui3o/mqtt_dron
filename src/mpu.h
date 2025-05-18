#ifndef MPU_H
#define MPU_H

#include <MadgwickAHRS.h>

extern Madgwick filter;  // Declare filter as extern
extern float mx, my, mz; // Declare magnetometer variables as extern

void kalmanUpdateRoll(float accAngleRoll, float gyroRateRoll);
void kalmanUpdatePitch(float accAnglePitch, float gyroRatePitch);
void setupMPU();
void loop_yaw();
void gyro_signals();
void calibrateSensors();
void meansensors();
void calibration();

#endif
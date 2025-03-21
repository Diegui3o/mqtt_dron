#ifndef MPU_H
#define MPU_H



#include <ESP32Servo.h>

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
void mpu_signals();
void pin_mode();


void setupMPU();
void loopMPU();
void gyro_signals();

#endif
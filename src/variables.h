#ifndef VARIABLES_H
#define VARIABLES_H

#include <Arduino.h>
#include <ESP32Servo.h>

extern volatile float RatePitch, RateRoll, RateYaw;
extern float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw, AccXCalibration, AccYCalibration, AccZCalibration;
extern int pinLed;

extern int ESCfreq;
extern float PAngleRoll, PAnglePitch;
extern float IAngleRoll, IAnglePitch;
extern float DAngleRoll, DAnglePitch;

extern float PRateRoll;
extern float IRateRoll;
extern float DRateRoll;

extern float PRatePitch;
extern float IRatePitch;
extern float DRatePitch;

extern float PRateYaw;
extern float IRateYaw;
extern float DRateYaw;

extern uint32_t LoopTimer;
extern float t;

extern volatile float AnglePitch_est;
extern volatile float AngleRoll_est;

extern Servo mot1;
extern Servo mot2;
extern Servo mot3;
extern Servo mot4;

extern const int mot1_pin;
extern const int mot2_pin;
extern const int mot3_pin;
extern const int mot4_pin;

extern volatile uint32_t current_time;
extern volatile uint32_t last_channel_1;
extern volatile uint32_t last_channel_2;
extern volatile uint32_t last_channel_3;
extern volatile uint32_t last_channel_4;
extern volatile uint32_t last_channel_5;
extern volatile uint32_t last_channel_6;
extern volatile uint32_t timer_1;
extern volatile uint32_t timer_2;
extern volatile uint32_t timer_3;
extern volatile uint32_t timer_4;
extern volatile uint32_t timer_5;
extern volatile uint32_t timer_6;
extern volatile int ReceiverValue[6];
extern const int channel_1_pin;
extern const int channel_2_pin;
extern const int channel_3_pin;
extern const int channel_4_pin;
extern const int channel_5_pin;
extern const int channel_6_pin;

extern volatile float PtermRoll;
extern volatile float ItermRoll;
extern volatile float DtermRoll;
extern volatile float PIDOutputRoll;
extern volatile float PtermPitch;
extern volatile float ItermPitch;
extern volatile float DtermPitch;
extern volatile float PIDOutputPitch;
extern volatile float PtermYaw;
extern volatile float ItermYaw;
extern volatile float DtermYaw;
extern volatile float PIDOutputYaw;

extern int ThrottleIdle;
extern int ThrottleCutOff;

extern volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
extern volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
extern volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
extern volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
extern volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
extern volatile float PIDReturn[3];

// Kalman filters for angle mode
extern volatile float AccX, AccY, AccZ;
extern volatile float GyroXdps, GyroYdps, GyroZdps;
extern volatile float AngleRoll, AnglePitch, AngleYaw;
extern volatile float KalmanAngleRoll, KalmanUncertaintyAngleRoll;
extern volatile float KalmanAnglePitch, KalmanUncertaintyAnglePitch;
extern volatile float Kalman1DOutput[2];
extern volatile float DesiredAngleRoll, DesiredAnglePitch;
extern volatile float ErrorAngleRoll, ErrorAnglePitch;
extern volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
extern volatile float PrevItermAngleRoll, PrevItermAnglePitch;

extern float complementaryAngleRoll;
extern float complementaryAnglePitch;

extern volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

#endif
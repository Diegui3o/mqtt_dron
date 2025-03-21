#include "variables.h"

volatile float RatePitch = 0.0, RateRoll = 0.0, RateYaw = 0.0;
float RateCalibrationPitch = 0.0, RateCalibrationRoll = 0.0, RateCalibrationYaw = 0.0;
float AccXCalibration = 0.0, AccYCalibration = 0.0, AccZCalibration = 0.0;
int pinLed = 2;

int ESCfreq = 500;
float PAngleRoll = 1.2, PAnglePitch = 1.2;
float IAngleRoll = 0.1, IAnglePitch = 0.1;
float DAngleRoll = 0.04, DAnglePitch = 0.04;

float PRateRoll = 0.625;
float IRateRoll = 2.1;
float DRateRoll = 0.0088;

float PRatePitch = 0.625;
float IRatePitch = 2.1;
float DRatePitch = 0.0088;

float PRateYaw = 4.0;
float IRateYaw = 3.0;
float DRateYaw = 0.0;

volatile float AngleRoll_est;
volatile float AnglePitch_est;


uint32_t LoopTimer;
float t = 0.004;

Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;

const int mot1_pin = 13;
const int mot2_pin = 12;
const int mot3_pin = 14;
const int mot4_pin = 27;

volatile uint32_t current_time;
volatile uint32_t last_channel_1 = 0;
volatile uint32_t last_channel_2 = 0;
volatile uint32_t last_channel_3 = 0;
volatile uint32_t last_channel_4 = 0;
volatile uint32_t last_channel_5 = 0;
volatile uint32_t last_channel_6 = 0;
volatile uint32_t timer_1;
volatile uint32_t timer_2;
volatile uint32_t timer_3;
volatile uint32_t timer_4;
volatile uint32_t timer_5;
volatile uint32_t timer_6;
volatile int ReceiverValue[6];
const int channel_1_pin = 34;
const int channel_2_pin = 35;
const int channel_3_pin = 32;
const int channel_4_pin = 33;
const int channel_5_pin = 25;
const int channel_6_pin = 26;

volatile float PtermRoll;
volatile float ItermRoll;
volatile float DtermRoll;
volatile float PIDOutputRoll;
volatile float PtermPitch;
volatile float ItermPitch;
volatile float DtermPitch;
volatile float PIDOutputPitch;
volatile float PtermYaw;
volatile float ItermYaw;
volatile float DtermYaw;
volatile float PIDOutputYaw;
volatile float KalmanGainPitch;
volatile float KalmanGainRoll;

int ThrottleIdle = 1050;
int ThrottleCutOff = 1000;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[3]={0,0,0};

// Kalman filters for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch, AngleYaw;
volatile float GyroXdps, GyroYdps, GyroZdps;
volatile float KalmanAngleRoll = 0.0, KalmanUncertaintyAngleRoll = 4.0;
volatile float KalmanAnglePitch = 0.0, KalmanUncertaintyAnglePitch = 4.0;
volatile float Kalman1DOutput[2]={0,0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
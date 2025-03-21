#include <Arduino.h>
#include "variables.h" 

void PIDequation()
{
    // Inlined PID equation for Roll
    ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
    PtermRoll = PAngleRoll * ErrorAngleRoll;
    ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
    ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
    DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
    PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
    PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
    DesiredRateRoll = PIDOutputRoll;
    PrevErrorAngleRoll = ErrorAngleRoll;
    PrevItermAngleRoll = ItermRoll;

    ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
    PtermPitch = PAnglePitch * ErrorAnglePitch;
    ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
    ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
    DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
    PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
    PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
    DesiredRatePitch = PIDOutputPitch;
    PrevErrorAnglePitch = ErrorAnglePitch;
    PrevItermAnglePitch = ItermPitch;

    // Compute errors
    ErrorRateRoll = DesiredRateRoll - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw = DesiredRateYaw - RateYaw;

    // Roll Axis PID
    PtermRoll = PRateRoll * ErrorRateRoll;
    ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
    ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
    DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
    PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
    PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);

    // Update output and previous values for Roll
    InputRoll = PIDOutputRoll;
    PrevErrorRateRoll = ErrorRateRoll;
    PrevItermRateRoll = ItermRoll;

    // Pitch Axis PID
    PtermPitch = PRatePitch * ErrorRatePitch;
    ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
    ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
    DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
    PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
    PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);

    // Update output and previous values for Pitch
    InputPitch = PIDOutputPitch;
    PrevErrorRatePitch = ErrorRatePitch;
    PrevItermRatePitch = ItermPitch;

    // Yaw Axis PID
    PtermYaw = PRateYaw * ErrorRateYaw;
    ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
    ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw);  // Clamp ItermYaw to [-400, 400]
    DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
    PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
    PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw);  // Clamp PIDOutputYaw to [-400, 400]
}

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
  float Pterm = P * Error;
  float Iterm = PrevIterm +( I * (Error + PrevError) * (t/2));
  if (Iterm > 400)
  {
    Iterm = 400;
  }
  else if (Iterm < -400)
  {
  Iterm = -400;
  }
  float Dterm = D *( (Error - PrevError)/t);
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400)
  {
    PIDOutput = 400;
  }
  else if (PIDOutput < -400)
  {
    PIDOutput = -400;
  }
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include "variables.h" 

void PIDequation();
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm);

#endif
#ifndef PILOTE_MODE_H
#define PILOTE_MODE_H

void setup_pilote_mode();
void loop_pilote_mode();
void calibrateSensors();
void setupMotores_pilote();
void applyControl(float tau_x);
void meansensors();
void calibration();

#endif
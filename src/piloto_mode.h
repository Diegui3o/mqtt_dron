#ifndef PILOTE_MODE_H
#define PILOTE_MODE_H

void setup_pilote_mode();
void loop_pilote_mode();
void calibrateSensors();
<<<<<<< HEAD
void applyControl(float tau_x, float tau_y, float tau_z);
void meansensors();
void calibration();

#endif // PILOTE_MODE_H
=======
void setupMotores_pilote();
void applyControl(float tau_x);
void meansensors();
void calibration();

#endif
>>>>>>> b1f65c3d56428ed493e4ce8dedd9ed69bd1c07f5

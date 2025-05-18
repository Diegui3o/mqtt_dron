#ifndef MOTORES_H
#define MOTORES_H

void updateDroneState();
void setupMotores();
void encenderMotores(int speed);
void apagarMotores();
void loopMotores();
void applyControl(float tau_x, float tau_y, float tau_z);
void applyIKZControl(float tau_x, float tau_y, float tau_z, float dt); // Prototipo para IKZ

#endif
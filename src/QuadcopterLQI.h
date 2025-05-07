// QuadcopterLQI.h
#ifndef QUADCOPTER_LQI_H
#define QUADCOPTER_LQI_H

#include <Arduino.h>

// Enum para definir el modo de control
enum class ControlMode
{
    DECOUPLED, // Control independiente por eje
    MIMO       // Control multivariable (acoplado)
};

// Estructura de parámetros del controlador
struct ControlParameters
{
    // Parámetros modo desacoplado (por eje)
    float rollKp; // Ganancia proporcional para roll
    float rollKi; // Ganancia integral para roll
    float rollKd; // Ganancia derivativa para roll

    float pitchKp; // Ganancia proporcional para pitch
    float pitchKi; // Ganancia integral para pitch
    float pitchKd; // Ganancia derivativa para pitch

    float yawKp; // Ganancia proporcional para yaw
    float yawKi; // Ganancia integral para yaw
    float yawKd; // Ganancia derivativa para yaw

    // Matriz K para modo MIMO (3×9)
    float K[3][9];

    // Constructor con valores predeterminados
    ControlParameters();
};

class QuadcopterLQI
{
public:
    // Constructor
    QuadcopterLQI(ControlMode mode = ControlMode::DECOUPLED);

    // Inicializar el controlador
    bool init();

    // Actualizar el controlador con el estado actual
    bool update(float *state, float dt);

    // Obtener salidas de control
    void getControlOutputs(float *outputs);

    // Establecer ángulos deseados
    void setDesiredAttitude(float roll, float pitch, float yaw);

    // Cambiar modo de control
    void setControlMode(ControlMode mode);

    // Obtener modo de control actual
    ControlMode getControlMode() const;

    // Actualizar parámetros del controlador
    void updateParameters(const ControlParameters &params);

    // Obtener parámetros actuales
    ControlParameters getParameters() const;

private:
    // Modo de control
    ControlMode _mode;
    ControlParameters _params;

    // Ángulos deseados
    float _desiredRoll;
    float _desiredPitch;
    float _desiredYaw;

    // Señales de control
    float _rollOutput;
    float _pitchOutput;
    float _yawOutput;

    // Estados de integrador
    float _rollIntegral;
    float _pitchIntegral;
    float _yawIntegral;

    // Límite de integrador (anti-windup)
    float _integralLimit;

    // Actualizar usando controlador desacoplado
    void updateDecoupled(float *state, float dt);

    // Actualizar usando controlador MIMO
    void updateMIMO(float *state, float dt);
};

#endif // QUADCOPTER_LQI_H
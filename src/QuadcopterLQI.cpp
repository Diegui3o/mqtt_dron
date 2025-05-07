// QuadcopterLQI.cpp
#include "QuadcopterLQI.h"

// Implementación del constructor de ControlParameters con valores por defecto
ControlParameters::ControlParameters()
    // Valores optimizados basados en el documento
    : rollKp(28.8910f), rollKi(31.6228f), rollKd(10.5624f),
      pitchKp(28.8910f), pitchKi(31.6228f), pitchKd(10.5624f),
      yawKp(3.0f), yawKi(1.0f), yawKd(0.8f)
{
    // Inicializar la matriz K para MIMO con valores optimizados

    // Primera fila (control roll)
    K[0][0] = 0.7f;  // roll error - phi_error
    K[0][1] = 0.1f;  // pitch error - theta_error (acople)
    K[0][2] = 0.0f;  // yaw error - psi_error
    K[0][3] = 0.3f;  // roll rate - phi_dot
    K[0][4] = 0.05f; // pitch rate - theta_dot (acople)
    K[0][5] = 0.0f;  // yaw rate - psi_dot
    K[0][6] = 0.2f;  // roll integral - ∫phi_error
    K[0][7] = 0.02f; // pitch integral - ∫theta_error (acople)
    K[0][8] = 0.0f;  // yaw integral - ∫psi_error

    // Segunda fila (control pitch)
    K[1][0] = 0.1f;  // roll error - phi_error (acople)
    K[1][1] = 0.7f;  // pitch error - theta_error
    K[1][2] = 0.0f;  // yaw error - psi_error
    K[1][3] = 0.05f; // roll rate - phi_dot (acople)
    K[1][4] = 0.3f;  // pitch rate - theta_dot
    K[1][5] = 0.0f;  // yaw rate - psi_dot
    K[1][6] = 0.02f; // roll integral - ∫phi_error (acople)
    K[1][7] = 0.2f;  // pitch integral - ∫theta_error
    K[1][8] = 0.0f;  // yaw integral - ∫psi_error

    // Tercera fila (control yaw)
    K[2][0] = 0.0f; // roll error - phi_error
    K[2][1] = 0.0f; // pitch error - theta_error
    K[2][2] = 0.5f; // yaw error - psi_error
    K[2][3] = 0.0f; // roll rate - phi_dot
    K[2][4] = 0.0f; // pitch rate - theta_dot
    K[2][5] = 0.2f; // yaw rate - psi_dot
    K[2][6] = 0.0f; // roll integral - ∫phi_error
    K[2][7] = 0.0f; // pitch integral - ∫theta_error
    K[2][8] = 0.1f; // yaw integral - ∫psi_error
}

QuadcopterLQI::QuadcopterLQI(ControlMode mode)
    : _mode(mode), _desiredRoll(0.0f), _desiredPitch(0.0f), _desiredYaw(0.0f),
      _rollOutput(0.0f), _pitchOutput(0.0f), _yawOutput(0.0f),
      _rollIntegral(0.0f), _pitchIntegral(0.0f), _yawIntegral(0.0f),
      _integralLimit(1.0f) // Límite de anti-windup
{
    // Constructor
}

bool QuadcopterLQI::init()
{
    // Resetear estados del controlador
    _rollIntegral = 0.0f;
    _pitchIntegral = 0.0f;
    _yawIntegral = 0.0f;

    _rollOutput = 0.0f;
    _pitchOutput = 0.0f;
    _yawOutput = 0.0f;

    // Inicializar con parámetros por defecto
    _params = ControlParameters();

    return true;
}

bool QuadcopterLQI::update(float *state, float dt)
{
    // Actualizar estados de integradores usando los errores de entrada
    // state[0..2] son los errores de ángulo

    // Actualizar integral de roll
    _rollIntegral += state[0] * dt;
    _rollIntegral = constrain(_rollIntegral, -_integralLimit, _integralLimit);

    // Actualizar integral de pitch
    _pitchIntegral += state[1] * dt;
    _pitchIntegral = constrain(_pitchIntegral, -_integralLimit, _integralLimit);

    // Actualizar integral de yaw
    _yawIntegral += state[2] * dt;
    _yawIntegral = constrain(_yawIntegral, -_integralLimit, _integralLimit);

    // Actualizar vector de estado con valores de integral actuales
    state[6] = _rollIntegral;
    state[7] = _pitchIntegral;
    state[8] = _yawIntegral;

    // Aplicar la estrategia de control apropiada
    if (_mode == ControlMode::DECOUPLED)
    {
        updateDecoupled(state, dt);
    }
    else
    {
        updateMIMO(state, dt);
    }

    return true;
}

void QuadcopterLQI::updateDecoupled(float *state, float dt)
{
    // Implementación del control LQI desacoplado (3 controladores independientes)
    // Como se describe en el documento, para cada eje implementamos un LQI
    // que considera el vector de estado:
    // x_φ = [φ_error, φ̇, ∫φ_error], x_θ = [θ_error, θ̇, ∫θ_error], x_ψ = [ψ_error, ψ̇, ∫ψ_error]

    // Control de roll (φ)
    // Vector de estado: [φ_error, φ̇, ∫φ_error]
    float rollState[3] = {state[0], state[3], _rollIntegral};
    _rollOutput = 0.0f;
    // Aplicamos K_φ ∈ R^1×3
    _rollOutput += _params.rollKp * rollState[0]; // Error de ángulo
    _rollOutput += _params.rollKd * rollState[1]; // Velocidad angular
    _rollOutput += _params.rollKi * rollState[2]; // Integral del error

    // Control de pitch (θ)
    // Vector de estado: [θ_error, θ̇, ∫θ_error]
    float pitchState[3] = {state[1], state[4], _pitchIntegral};
    _pitchOutput = 0.0f;
    // Aplicamos K_θ ∈ R^1×3
    _pitchOutput += _params.pitchKp * pitchState[0]; // Error de ángulo
    _pitchOutput += _params.pitchKd * pitchState[1]; // Velocidad angular
    _pitchOutput += _params.pitchKi * pitchState[2]; // Integral del error

    // Control de yaw (ψ)
    // Vector de estado: [ψ_error, ψ̇, ∫ψ_error]
    float yawState[3] = {state[2], state[5], _yawIntegral};
    _yawOutput = 0.0f;
    // Aplicamos K_ψ ∈ R^1×3
    _yawOutput += _params.yawKp * yawState[0]; // Error de ángulo
    _yawOutput += _params.yawKd * yawState[1]; // Velocidad angular
    _yawOutput += _params.yawKi * yawState[2]; // Integral del error

    // Limitamos las salidas al rango válido para evitar saturación
    _rollOutput = constrain(_rollOutput, -1.0f, 1.0f);
    _pitchOutput = constrain(_pitchOutput, -1.0f, 1.0f);
    _yawOutput = constrain(_yawOutput, -1.0f, 1.0f);
}

void QuadcopterLQI::updateMIMO(float *state, float dt)
{
    // Implementación de LQI MIMO (Linear Quadratic Integral)
    // Enfoque multivariable que captura los acoples entre ejes
    // Como se describe en el documento, utilizamos un modelo de
    // estado aumentado con integrador en cada eje

    // El vector de estado completo es:
    // x = [φ_error, θ_error, ψ_error, φ̇, θ̇, ψ̇, ∫φ_error, ∫θ_error, ∫ψ_error]
    // Y la ley de control es u = -K·x donde K ∈ R^3×9

    // Inicializar salidas a cero
    _rollOutput = 0.0f;
    _pitchOutput = 0.0f;
    _yawOutput = 0.0f;

    // Multiplicación de matrices (3x9 * 9x1 = 3x1)
    // Aplicamos la ley de control óptimo LQI calculada previamente
    for (int i = 0; i < 9; i++)
    {
        _rollOutput -= _params.K[0][i] * state[i];
        _pitchOutput -= _params.K[1][i] * state[i];
        _yawOutput -= _params.K[2][i] * state[i];
    }

    // Aplicamos limitación a las salidas para evitar saturación
    _rollOutput = constrain(_rollOutput, -1.0f, 1.0f);
    _pitchOutput = constrain(_pitchOutput, -1.0f, 1.0f);
    _yawOutput = constrain(_yawOutput, -1.0f, 1.0f);
}

void QuadcopterLQI::getControlOutputs(float *outputs)
{
    outputs[0] = _rollOutput;
    outputs[1] = _pitchOutput;
    outputs[2] = _yawOutput;
}

void QuadcopterLQI::setDesiredAttitude(float roll, float pitch, float yaw)
{
    _desiredRoll = roll;
    _desiredPitch = pitch;
    _desiredYaw = yaw;
}

void QuadcopterLQI::setControlMode(ControlMode mode)
{
    // Al cambiar de modo, resetear estados integrales para evitar saltos
    _rollIntegral = 0.0f;
    _pitchIntegral = 0.0f;
    _yawIntegral = 0.0f;

    _mode = mode;
}

ControlMode QuadcopterLQI::getControlMode() const
{
    return _mode;
}

void QuadcopterLQI::updateParameters(const ControlParameters &params)
{
    _params = params;
}

ControlParameters QuadcopterLQI::getParameters() const
{
    return _params;
}
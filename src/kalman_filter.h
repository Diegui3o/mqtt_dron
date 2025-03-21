#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// Declaración del filtro de Kalman
struct KalmanFilter {
    float Q, R, A, B, C, x, cov;
    KalmanFilter(float q, float r, float a, float b, float c) : Q(q), R(r), A(a), B(b), C(c), x(0), cov(1) {}
    void predict(float u) { x = A * x + B * u; cov = A * cov * A + Q; }
    void update(float z) { float K = cov * C / (C * cov * C + R); x = x + K * (z - C * x); cov = (1 - K * C) * cov; }
    float getState() { return x; }
  };

#endif // KALMAN_FILTER_H
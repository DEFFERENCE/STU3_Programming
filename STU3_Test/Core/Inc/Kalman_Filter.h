#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "arm_math.h"

#define KALMAN_STATE_DIM 4
#define KALMAN_MEAS_DIM  4

typedef struct {
    arm_matrix_instance_f32 A, B, H, Q, R, P, K;
    arm_matrix_instance_f32 x, u, z;
    arm_matrix_instance_f32 temp1, temp2, temp3;

    float32_t A_data[KALMAN_STATE_DIM * KALMAN_STATE_DIM];
    float32_t B_data[KALMAN_STATE_DIM * KALMAN_MEAS_DIM];
    float32_t H_data[KALMAN_MEAS_DIM * KALMAN_STATE_DIM];
    float32_t Q_data[KALMAN_STATE_DIM * KALMAN_STATE_DIM];
    float32_t R_data[KALMAN_MEAS_DIM * KALMAN_MEAS_DIM];
    float32_t P_data[KALMAN_STATE_DIM * KALMAN_STATE_DIM];
    float32_t K_data[KALMAN_STATE_DIM * KALMAN_MEAS_DIM];

    float32_t x_data[KALMAN_STATE_DIM];
    float32_t u_data[KALMAN_MEAS_DIM];
    float32_t z_data[KALMAN_MEAS_DIM];

    float32_t temp1_data[KALMAN_STATE_DIM * KALMAN_STATE_DIM];
    float32_t temp2_data[KALMAN_STATE_DIM * KALMAN_MEAS_DIM];
    float32_t temp3_data[KALMAN_MEAS_DIM * KALMAN_MEAS_DIM];
} KalmanFilter;

void Kalman_Init(KalmanFilter *kf);
void Kalman_Predict(KalmanFilter *kf);
void Kalman_Update(KalmanFilter *kf, float32_t *measurement);
void Kalman_SetProcessNoise(KalmanFilter *kf, float value);
void Kalman_SetMeasurementNoise(KalmanFilter *kf, float value);

#endif

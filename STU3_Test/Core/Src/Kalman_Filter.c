#include "Kalman_Filter.h"

void Kalman_Init(KalmanFilter *kf) {
    arm_mat_init_f32(&kf->A, KALMAN_STATE_DIM, KALMAN_STATE_DIM, kf->A_data);
    arm_mat_init_f32(&kf->B, KALMAN_STATE_DIM, KALMAN_MEAS_DIM, kf->B_data);
    arm_mat_init_f32(&kf->H, KALMAN_MEAS_DIM, KALMAN_STATE_DIM, kf->H_data);
    arm_mat_init_f32(&kf->Q, KALMAN_STATE_DIM, KALMAN_STATE_DIM, kf->Q_data);
    arm_mat_init_f32(&kf->R, KALMAN_MEAS_DIM, KALMAN_MEAS_DIM, kf->R_data);
    arm_mat_init_f32(&kf->P, KALMAN_STATE_DIM, KALMAN_STATE_DIM, kf->P_data);
    arm_mat_init_f32(&kf->K, KALMAN_STATE_DIM, KALMAN_MEAS_DIM, kf->K_data);

    arm_mat_init_f32(&kf->x, KALMAN_STATE_DIM, 1, kf->x_data);
    arm_mat_init_f32(&kf->u, KALMAN_MEAS_DIM, 1, kf->u_data);
    arm_mat_init_f32(&kf->z, KALMAN_MEAS_DIM, 1, kf->z_data);

    arm_mat_init_f32(&kf->temp1, KALMAN_STATE_DIM, KALMAN_STATE_DIM, kf->temp1_data);
    arm_mat_init_f32(&kf->temp2, KALMAN_STATE_DIM, KALMAN_MEAS_DIM, kf->temp2_data);
    arm_mat_init_f32(&kf->temp3, KALMAN_MEAS_DIM, KALMAN_MEAS_DIM, kf->temp3_data);

    // Clear state
    for (int i = 0; i < KALMAN_STATE_DIM; i++) {
        kf->x_data[i] = 0.0f;
    }
}

void Kalman_SetProcessNoise(KalmanFilter *kf, float value) {
    for (int i = 0; i < KALMAN_STATE_DIM; i++) {
        for (int j = 0; j < KALMAN_STATE_DIM; j++) {
            kf->Q_data[i * KALMAN_STATE_DIM + j] = (i == j) ? value : 0.0f;
        }
    }
}

void Kalman_SetMeasurementNoise(KalmanFilter *kf, float value) {
    for (int i = 0; i < KALMAN_MEAS_DIM; i++) {
        for (int j = 0; j < KALMAN_MEAS_DIM; j++) {
            kf->R_data[i * KALMAN_MEAS_DIM + j] = (i == j) ? value : 0.0f;
        }
    }
}

void Kalman_Predict(KalmanFilter *kf) {
    // x = A * x
    arm_mat_mult_f32(&kf->A, &kf->x, &kf->x);

    // P = A * P * A' + Q
    arm_mat_mult_f32(&kf->A, &kf->P, &kf->temp1);
    arm_mat_trans_f32(&kf->A, &kf->temp2);  // reuse temp2
    arm_mat_mult_f32(&kf->temp1, &kf->temp2, &kf->P);
    arm_mat_add_f32(&kf->P, &kf->Q, &kf->P);
}

void Kalman_Update(KalmanFilter *kf, float32_t *measurement) {
    for (int i = 0; i < KALMAN_MEAS_DIM; i++) {
        kf->z_data[i] = measurement[i];
    }

    // K = P * H' * (H * P * H' + R)^-1
    arm_mat_trans_f32(&kf->H, &kf->temp2);
    arm_mat_mult_f32(&kf->P, &kf->temp2, &kf->temp1);
    arm_mat_mult_f32(&kf->H, &kf->temp1, &kf->temp3);
    arm_mat_add_f32(&kf->temp3, &kf->R, &kf->temp3);
    arm_mat_inverse_f32(&kf->temp3, &kf->temp3);
    arm_mat_mult_f32(&kf->temp1, &kf->temp3, &kf->K);

    // x = x + K * (z - H * x)
    arm_mat_mult_f32(&kf->H, &kf->x, &kf->u);         // reuse u for Hx
    for (int i = 0; i < KALMAN_MEAS_DIM; i++) {
        kf->u_data[i] = kf->z_data[i] - kf->u_data[i]; // z - Hx
    }
    arm_mat_mult_f32(&kf->K, &kf->u, &kf->u);         // reuse u for K*(z-Hx)
    for (int i = 0; i < KALMAN_STATE_DIM; i++) {
        kf->x_data[i] += kf->u_data[i];
    }

    // P = (I - K * H) * P
    for (int i = 0; i < KALMAN_STATE_DIM * KALMAN_STATE_DIM; i++) {
        kf->temp1_data[i] = 0.0f;
    }
    for (int i = 0; i < KALMAN_STATE_DIM; i++) {
        kf->temp1_data[i * KALMAN_STATE_DIM + i] = 1.0f;
    }
    arm_mat_mult_f32(&kf->K, &kf->H, &kf->temp2);
    arm_mat_sub_f32(&kf->temp1, &kf->temp2, &kf->temp1);
    arm_mat_mult_f32(&kf->temp1, &kf->P, &kf->P);
}

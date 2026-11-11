#include "kalman.h"
#include "matrix.h"
#include <math.h>

static float state_estimate[4][1] = { {0.0f}, {0.0f}, {0.0f}, {0.0f} };

static float P[4][4] = {
    {1.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 1.0f}
};

// Process noise covariance Q (tuned for flight dynamics)
static const float Q[4][4] = {
    {0.001f, 0.0f,    0.0f,    0.0f},
    {0.0f,   0.0001f, 0.0f,    0.0f},
    {0.0f,   0.0f,    0.003f,  0.0f},
    {0.0f,   0.0f,    0.0f,    0.0001f}
};

// Measurement noise covariance R
static const float R[2][2] = {
    {0.05f, 0.0f},
    {0.0f,  0.05f}
};

// Measurement Matrix H
static const float H[2][4] = {
    {1.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, 0.0f}
};

static const float I_mat[4][4] = {
    {1.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 1.0f}
};

void Kalman_Init(void) {
    for (int i = 0; i < 4; i++) {
        state_estimate[i][0] = 0.0f;
        for (int j = 0; j < 4; j++) {
            P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

void Kalman_Update(IMU_Data_t *imu, Kalman_State_t *state, float dt) {
    float p = imu->gx;
    float q = imu->gy;
    float r = imu->gz;

    float phi   = state_estimate[0][0];
    float theta = state_estimate[2][0];

    float tan_theta = tanf(theta);
    if (tan_theta > 10.0f)  tan_theta = 10.0f;
    if (tan_theta < -10.0f) tan_theta = -10.0f;

    float phi_dot   = p + sinf(phi) * tan_theta * q + cosf(phi) * tan_theta * r;
    float theta_dot = cosf(phi) * q - sinf(phi) * r;

    float gyro_input[2][1] = { {phi_dot}, {theta_dot} };

    // State Transition Matrix F and Control Input Matrix G
    float F[4][4] = {
        {1.0f, -dt,   0.0f, 0.0f},
        {0.0f, 1.0f,  0.0f, 0.0f},
        {0.0f, 0.0f,  1.0f, -dt},
        {0.0f, 0.0f,  0.0f, 1.0f}
    };

    float G[4][2] = {
        {dt,   0.0f},
        {0.0f, 0.0f},
        {0.0f, dt},
        {0.0f, 0.0f}
    };

    // 1. Prediction Step
    float F_x[4][1], G_u[4][1], x_pred[4][1];
    mat_mult((float*)F, 4, 4, (float*)state_estimate, 4, 1, (float*)F_x);
    mat_mult((float*)G, 4, 2, (float*)gyro_input, 2, 1, (float*)G_u);
    mat_add((float*)F_x, (float*)G_u, 4, 1, (float*)x_pred);

    float FT[4][4], FP[4][4], FPF_T[4][4], P_pred[4][4];
    mat_trans((float*)F, 4, 4, (float*)FT);
    mat_mult((float*)F, 4, 4, (float*)P, 4, 4, (float*)FP);
    mat_mult((float*)FP, 4, 4, (float*)FT, 4, 4, (float*)FPF_T);
    mat_add((float*)FPF_T, (float*)Q, 4, 4, (float*)P_pred);

    // 2. Measurement Update Step (Correction)
    float acc_roll  = atan2f(imu->ay, imu->az);
    float acc_pitch = atan2f(imu->ax, sqrtf(imu->ay * imu->ay + imu->az * imu->az));
    float measurement[2][1] = { {acc_roll}, {acc_pitch} };

    float H_x[2][1], y_tilde[2][1];
    mat_mult((float*)H, 2, 4, (float*)x_pred, 4, 1, (float*)H_x);
    mat_sub((float*)measurement, (float*)H_x, 2, 1, (float*)y_tilde);

    float HT[4][2], HP[2][4], HPH_T[2][2], S[2][2];
    mat_trans((float*)H, 2, 4, (float*)HT);
    mat_mult((float*)H, 2, 4, (float*)P_pred, 4, 4, (float*)HP);
    mat_mult((float*)HP, 2, 4, (float*)HT, 4, 2, (float*)HPH_T);
    mat_add((float*)HPH_T, (float*)R, 2, 2, (float*)S);

    float invS[2][2];
    if (!mat_inv_2x2((float*)S, (float*)invS)) return;

    float P_HT[4][2], K[4][2];
    mat_mult((float*)P_pred, 4, 4, (float*)HT, 4, 2, (float*)P_HT);
    mat_mult((float*)P_HT, 4, 2, (float*)invS, 2, 2, (float*)K);

    // State update
    float K_y[4][1];
    mat_mult((float*)K, 4, 2, (float*)y_tilde, 2, 1, (float*)K_y);
    mat_add((float*)x_pred, (float*)K_y, 4, 1, (float*)state_estimate);

    // Covariance update (Joseph Form)
    float KH[4][4], I_KH[4][4], I_KH_T[4][4];
    float term1[4][4], temp_P[4][4];
    float KT[2][4], KR[4][2], KRK_T[4][4];

    mat_mult((float*)K, 4, 2, (float*)H, 2, 4, (float*)KH);
    mat_sub((float*)I_mat, (float*)KH, 4, 4, (float*)I_KH);
    mat_trans((float*)I_KH, 4, 4, (float*)I_KH_T);

    mat_mult((float*)I_KH, 4, 4, (float*)P_pred, 4, 4, (float*)temp_P);
    mat_mult((float*)temp_P, 4, 4, (float*)I_KH_T, 4, 4, (float*)term1);

    mat_trans((float*)K, 4, 2, (float*)KT);
    mat_mult((float*)K, 4, 2, (float*)R, 2, 2, (float*)KR);
    mat_mult((float*)KR, 4, 2, (float*)KT, 2, 4, (float*)KRK_T);

    mat_add((float*)term1, (float*)KRK_T, 4, 4, (float*)P);

    // 3. Output results in degrees
    state->roll       = state_estimate[0][0] * (180.0f / (float)M_PI);
    state->roll_bias  = state_estimate[1][0] * (180.0f / (float)M_PI);
    state->pitch      = state_estimate[2][0] * (180.0f / (float)M_PI);
    state->pitch_bias = state_estimate[3][0] * (180.0f / (float)M_PI);
}

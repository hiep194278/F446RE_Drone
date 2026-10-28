#include "complementary_filter.h"
#include <math.h>

static float phi_hat = 0.0f;
static float theta_hat = 0.0f;
static float alpha = 0.02f;

void ComplementaryFilter_Init(void)
{
    phi_hat = 0.0f;
    theta_hat = 0.0f;
}

void ComplementaryFilter_Update(IMU_Data_t *imu, Attitude_t *attitude, float dt)
{
    float phi_acc = atan2f(imu->ay, imu->az);
    float theta_acc = atan2f(imu->ax, sqrtf(imu->ay * imu->ay + imu->az * imu->az));

    float phi_dot = imu->gx + sinf(phi_hat) * tanf(theta_hat) * imu->gy + cosf(phi_hat) * tanf(theta_hat) * imu->gz;
    float theta_dot = cosf(phi_hat) * imu->gy - sinf(phi_hat) * imu->gz;

    phi_hat = (1.0f - alpha) * (phi_hat + dt * phi_dot) + alpha * phi_acc;
    theta_hat = (1.0f - alpha) * (theta_hat + dt * theta_dot) + alpha * theta_acc;

    attitude->roll = phi_hat * 180.0f / (float)M_PI;
    attitude->pitch = theta_hat * 180.0f / (float)M_PI;
}

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "main.h"
#include "imu.h"

typedef struct {
    float roll;
    float roll_bias;
    float pitch;
    float pitch_bias;
} Kalman_State_t;

void Kalman_Init(void);
void Kalman_Update(IMU_Data_t *imu, Kalman_State_t *state, float dt);

#endif /* KALMAN_FILTER_H_ */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
	float ax;
	float ay;
	float az;
    float gx;
    float gy;
    float gz;
} IMU_Data_t;

typedef struct {
    uint32_t success_count;
    uint32_t error_count;
    uint32_t recovery_count;
    uint32_t last_hal_status; // 0 = HAL_OK,
                              // 1 = HAL_ERROR,
                              // 2 = HAL_BUSY,
                              // 3 = HAL_TIMEOUT
} IMU_Diagnostics_t;

void IMU_Init(I2C_HandleTypeDef *hi2c);
bool IMU_Read(IMU_Data_t *imu);
void IMU_CalibrateGyro(void);
void IMU_GetDiagnostics(IMU_Diagnostics_t *diag);

#endif /* INC_IMU_H_ */

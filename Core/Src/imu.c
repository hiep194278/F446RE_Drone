#include "imu.h"
#include <math.h>

#define MPU_ADDR (0x68 << 1)

static float gx_bias = 0.0f;
static float gy_bias = 0.0f;
static float gz_bias = 0.0f;
static I2C_HandleTypeDef *imu_i2c;

static uint32_t s_success_count = 0;
static uint32_t s_error_count = 0;
static uint32_t s_recovery_count = 0;
static uint32_t s_last_hal_status = HAL_OK;

void IMU_Init(I2C_HandleTypeDef *hi2c)
{
    imu_i2c = hi2c;
    uint8_t data = 0x00;

    // Wake up MPU6050 (Power Management 1 register -> 0x6B)
    HAL_I2C_Mem_Write(imu_i2c, MPU_ADDR, 0x6B, 1, &data, 1, 10);
    HAL_Delay(50);

    // Configure Gyro (+/- 250 deg/s) and Accel (+/- 2g)
	data = 0x00;
	HAL_I2C_Mem_Write(imu_i2c, MPU_ADDR, 0x1B, 1, &data, 1, 10);
	HAL_I2C_Mem_Write(imu_i2c, MPU_ADDR, 0x1C, 1, &data, 1, 10);

	// Digital Low-Pass Filter (~42Hz)
	data = 0x03;
	HAL_I2C_Mem_Write(imu_i2c, MPU_ADDR, 0x1A, 1, &data, 1, 10);
	HAL_Delay(100);
}

bool IMU_Read(IMU_Data_t *imu)
{
    uint8_t buf[14];

    // Perform I2C memory read and store the exact HAL status code
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(imu_i2c, MPU_ADDR, 0x3B, 1, buf, 14, 5);
    s_last_hal_status = (uint32_t)status;

    if (status != HAL_OK)
    {
        s_error_count++;
        HAL_I2C_DeInit(imu_i2c);
        HAL_I2C_Init(imu_i2c);
        s_recovery_count++;
        return false;
    }

    s_success_count++;

    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);

    int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

    imu->ax = (float)ax / 16384.0f;
    imu->ay = (float)ay / 16384.0f;
    imu->az = (float)az / 16384.0f;

    imu->gx = ((float)gx / 131.0f) * (float)(M_PI / 180.0) - gx_bias;
    imu->gy = ((float)gy / 131.0f) * (float)(M_PI / 180.0) - gy_bias;
    imu->gz = ((float)gz / 131.0f) * (float)(M_PI / 180.0) - gz_bias;

    return true;
}

void IMU_CalibrateGyro(void)
{
	IMU_Data_t sample;
	float sx = 0.0f, sy = 0.0f, sz = 0.0f;
	const int N = 2000;
	int successful_samples = 0;

	// Reset biases before calculation
	gx_bias = 0.0f;
	gy_bias = 0.0f;
	gz_bias = 0.0f;

	for (int i = 0; i < N; i++) {
		if (IMU_Read(&sample)) {
			sx += sample.gx;
			sy += sample.gy;
			sz += sample.gz;
			successful_samples++;
		}
		HAL_Delay(2);
	}

	if (successful_samples > 0) {
		gx_bias = sx / successful_samples;
		gy_bias = sy / successful_samples;
		gz_bias = sz / successful_samples;
	}
}

void IMU_GetDiagnostics(IMU_Diagnostics_t *diag)
{
    diag->success_count = s_success_count;
    diag->error_count = s_error_count;
    diag->recovery_count = s_recovery_count;
    diag->last_hal_status = s_last_hal_status;
}

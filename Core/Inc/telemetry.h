#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "stm32f4xx_hal.h"
#include "imu.h"

#define PKT_HEADER_1       0xBB
#define PKT_HEADER_2       0x66
#define PKT_TYPE_ARM       0x01
#define PKT_TYPE_MODE      0x02
#define PKT_TYPE_RC        0x03
#define PKT_TYPE_CALIB     0x04

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
} Attitude_Data_t;

typedef struct {
    int16_t roll;
    int16_t pitch;
    int16_t throttle;
    int16_t yaw;
    bool armed;
    uint8_t flight_mode; // 0 = Stabilize, 1 = Acro
} FlightControl_t;

void Telemetry_Init(UART_HandleTypeDef *uart_handle);
void Telemetry_Send_Attitude(Attitude_Data_t *attitude);
void Telemetry_Start_Receive(void);
const FlightControl_t* Telemetry_Get_Control(void);

#endif

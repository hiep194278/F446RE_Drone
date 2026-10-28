#ifndef INC_COMPLEMENTARY_FILTER_H_
#define INC_COMPLEMENTARY_FILTER_H_

#include "imu.h"

typedef struct
{
    float roll;
    float pitch;

} Attitude_t; // ZYX Euler Sequence
              // Yaw -> Pitch -> Roll

void ComplementaryFilter_Init(void);

void ComplementaryFilter_Update(
    IMU_Data_t *imu,
    Attitude_t *attitude,
    float dt
);

#endif /* INC_COMPLEMENTARY_FILTER_H_ */

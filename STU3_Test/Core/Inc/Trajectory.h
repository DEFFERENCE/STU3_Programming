#ifndef INC_TRAJECTORY_H_
#define INC_TRAJECTORY_H_

#include "stm32g4xx_hal.h"

typedef struct {
    float distance;       // Total distance
    float v_max;          // Max velocity
    float a_max;          // Max acceleration
    float t_acc;          // Time for acceleration
    float t_flat;        // Time for constant velocity
    float t_total;        // Total time of the trajectory
} TrajectoryParams;

void InitTrajectory(float distance, float v_max, float a_max);
float GetPositionAtTime(float t);
float GetVelocityAtTime(float t);

#endif /* INC_TRAJECTORY_H_ */

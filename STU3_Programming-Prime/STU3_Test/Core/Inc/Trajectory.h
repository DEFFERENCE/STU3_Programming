#ifndef INC_TRAJECTORY_H_
#define INC_TRAJECTORY_H_

#include "stm32g4xx_hal.h"

#include <stdint.h>

#define MAX_SEGMENTS 10

typedef struct {
    float start_pos;
    float end_pos;
    float v_max;
    float a_max;
    float t_accel;
    float t_const;
    float t_decel;
    float t_total;
    float t_start;
} TrajectorySegment;

void InitTrajectorySegment(TrajectorySegment *seg, float start, float end, float v_max, float a_max, float t_start);
float GetTrajectoryPosition(const TrajectorySegment *seg, float t_global);
float GetTrajectoryVelocity(const TrajectorySegment *seg, float t_global);
void InitHoldTrajectorySegment(TrajectorySegment *seg, float pos, float duration, float t_start);

#endif /* INC_TRAJECTORY_H_ */

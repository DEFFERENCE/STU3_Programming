#include "Trajectory.h"
#include <math.h>

void InitTrajectorySegment(TrajectorySegment *seg, float start, float end, float v_max, float a_max, float t_start) {
    float D = end - start;
    float dir = (D >= 0) ? 1.0f : -1.0f;
    D = fabsf(D);

    float t_accel = v_max / a_max;
    float d_accel = 0.5f * a_max * t_accel * t_accel;

    if (2 * d_accel > D) {
        // Triangular profile
        t_accel = sqrtf(D / a_max);
        seg->t_const = 0;
        seg->t_total = 2 * t_accel;
    } else {
        // Trapezoidal profile
        float d_const = D - 2 * d_accel;
        seg->t_const = d_const / v_max;
        seg->t_total = 2 * t_accel + seg->t_const;
    }

    seg->start_pos = start;
    seg->end_pos = end;
    seg->v_max = v_max * dir;
    seg->a_max = a_max * dir;
    seg->t_accel = t_accel;
    seg->t_decel = t_accel;
    seg->t_start = t_start;
}

float GetTrajectoryPosition(const TrajectorySegment *seg, float t_global) {
    float t = t_global - seg->t_start;
    if (t < 0) return seg->start_pos;
    if (t >= seg->t_total) return seg->end_pos;

    float a = seg->a_max;
    float v = seg->v_max;
    float p0 = seg->start_pos;

    if (t < seg->t_accel) {
        return p0 + 0.5f * a * t * t;
    } else if (t < seg->t_accel + seg->t_const) {
        float t1 = seg->t_accel;
        float p1 = p0 + 0.5f * a * t1 * t1;
        return p1 + v * (t - t1);
    } else {
        float t1 = seg->t_accel;
        float t2 = seg->t_const;
        float p1 = p0 + 0.5f * a * t1 * t1;
        float p2 = p1 + v * t2;
        float td = t - t1 - t2;
        return p2 + v * td - 0.5f * a * td * td;
    }
}

float GetTrajectoryVelocity(const TrajectorySegment *seg, float t_global) {
    float t = t_global - seg->t_start;
    if (t < 0) return 0;
    if (t >= seg->t_total) return 0;

    float a = seg->a_max;
    float v = seg->v_max;

    if (t < seg->t_accel) {
        return a * t;
    } else if (t < seg->t_accel + seg->t_const) {
        return v;
    } else {
        float td = t - seg->t_accel - seg->t_const;
        return v - a * td;
    }
}

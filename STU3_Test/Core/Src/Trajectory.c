#include "Trajectory.h"
#include <math.h>

TrajectoryParams traj;

void InitTrajectory(float distance, float v_max, float a_max) {
    traj.distance = distance;
    traj.v_max = v_max;
    traj.a_max = a_max;

    traj.t_acc = v_max / a_max;
    float d_acc = 0.5 * a_max * traj.t_acc * traj.t_acc;

    if (2 * d_acc >= distance) {
        // Triangular profile - No constant speed
        traj.t_acc = sqrtf(distance / a_max);
        traj.t_flat = 0;
        traj.t_total = 2 * traj.t_acc;
    } else {
        float d_flat = distance - 2 * d_acc;
        traj.t_flat = d_flat / v_max;
        traj.t_total = 2 * traj.t_acc + traj.t_flat;
    }
}

float GetPositionAtTime(float t) {
    float a = traj.a_max;
    float v = traj.v_max;
    float t1 = traj.t_acc;
    float t2 = traj.t_flat;
    float x = 0.0;

    if (t < t1) {
        // Acceleration phase
        x = 0.5 * a * t * t;
    } else if (t < t1 + t2) {
        // Constant velocity
        x = 0.5 * a * t1 * t1 + v * (t - t1);
    } else if (t <= traj.t_total) {
        // Deceleration
        float td = t - t1 - t2;
        x = 0.5 * a * t1 * t1 + v * t2 + v * td - 0.5 * a * td * td;
    } else {
        // After the trajectory finished
        x = traj.distance;
    }

    return x;
}

float GetVelocityAtTime(float t) {
    float a = traj.a_max;
    float v = traj.v_max;
    float t1 = traj.t_acc;
    float t2 = traj.t_flat;
    float vel = 0.0;

    if (t < t1) {
        vel = a * t;
    } else if (t < t1 + t2) {
        vel = v;
    } else if (t <= traj.t_total) {
        float td = t - t1 - t2;
        vel = v - a * td;
    } else {
        vel = 0.0;
    }

    return vel;
}

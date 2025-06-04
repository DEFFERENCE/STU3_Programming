/*
 * Revolute.c
 *
 *  Created on: May 7, 2025
 *      Author: User
 */

#include <stdio.h>
#include "revolute.h"

float v_rev_dis[2] = {0, 0};
float v_rev_ref[3] = {0, 0, 0};

RevoluteMotor create_motor(double J, double B, double Eff, double Ke, double Kt, double R, double L) {
    RevoluteMotor motor = { J, B, Eff, Ke, Kt, R, L };
    return motor;
}

// Converts disturbance torque into equivalent voltage needed to cancel it
float disturbance_feedforward(const RevoluteMotor* motor, float load_torque) {
	float num = 0.3298 - (0.04473 * v_rev_dis[1]);
	float den = 1 - (0.3333 * v_rev_dis[1]);
	v_rev_dis[0] = (num/den) * load_torque;
	v_rev_dis[1] = v_rev_dis[0];
    return v_rev_dis[0];
}

// Calculates voltage needed to maintain a certain angular velocity
float reference_feedforward(const RevoluteMotor* motor, float angular_velocity) {
	float num = 4.678 - (5.295 * v_rev_ref[1]) + (0.6394 * v_rev_ref[2]);
	float den = 1 - (1.81 * v_rev_ref[1]) + (0.8186 * v_rev_ref[2]);
	v_rev_ref[0] = (num/den) * angular_velocity;
	v_rev_ref[2] = v_rev_ref[1];
	v_rev_ref[1] = v_rev_ref[0];
	return v_rev_ref[0];
}

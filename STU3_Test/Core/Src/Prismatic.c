/*
 * Prismatic.c
 *
 *  Created on: May 7, 2025
 *      Author: User
 */

#include <stdio.h>
#include "Prismatic.h"

float v_pris_dis[2] = {0, 0};
float v_pris_ref[3] = {0, 0, 0};

PrismaticMotor create_prismatic_motor(double J, double B, double Eff, double Ke,
		double Kt, double R, double L) {
	PrismaticMotor motor = { J, B, Eff, Ke, Kt, R, L };
	return motor;
}

float disturbance_feedforward_pri(const PrismaticMotor *motor, float load_force) {
	float num = 0.3368 + (0.032 * v_pris_dis[1]);
	float den = 1 - (0.9048 * v_pris_dis[1]);
	v_pris_dis[0] = (num/den) * load_force;
	v_pris_dis[1] = v_pris_dis[0];
	return v_pris_dis[0];
}

float reference_feedforward_pri(const PrismaticMotor *motor, float linear_velocity) {
	float num = 0.01191 + (0.01774 * v_pris_ref[1]) + (0.009618 * v_pris_ref[2]);
	float den = 1 - (1.81 * v_pris_ref[1]) + (0.8186 * v_pris_ref[2]);
	v_pris_ref[0] = (num/den) * linear_velocity;
	v_pris_ref[2] = v_pris_ref[1];
	v_pris_ref[1] = v_pris_ref[0];
	return v_pris_ref[0];
}

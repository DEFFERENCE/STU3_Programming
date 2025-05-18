/*
 * Prismatic.c
 *
 *  Created on: May 7, 2025
 *      Author: User
 */
#include <stdio.h>
#include "Prismatic.h"

PrismaticMotor create_prismatic_motor(double J, double B, double Eff, double Ke,
		double Kt, double R, double L) {
	PrismaticMotor motor = { J, B, Eff, Ke, Kt, R, L };
	return motor;
}

float disturbance_feedforward_pri(const PrismaticMotor *motor,
		double load_force) {
	float voltage = 0;
	return voltage;
}

float reference_feedforward_pri(const PrismaticMotor *motor,
		double linear_velocity) {
	float voltage = 0;
	return voltage;
}


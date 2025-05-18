/*
 * Revolute.c
 *
 *  Created on: May 7, 2025
 *      Author: User
 */

#include <stdio.h>
#include "revolute.h"

RevoluteMotor create_motor(double J, double B, double Eff, double Ke, double Kt, double R, double L) {
    RevoluteMotor motor = { J, B, Eff, Ke, Kt, R, L };
    return motor;
}


// Converts disturbance torque into equivalent voltage needed to cancel it
float disturbance_feedforward(const RevoluteMotor* motor, double load_torque) {
	float voltage = 0;
    return voltage;
}

// Calculates voltage needed to maintain a certain angular velocity
float reference_feedforward(const RevoluteMotor* motor, double angular_velocity) {
	float voltage = 0;
	return voltage;
}


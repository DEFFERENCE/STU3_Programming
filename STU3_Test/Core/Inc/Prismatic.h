/*
 * Prismatic.h
 *
 *  Created on: May 7, 2025
 *      Author: User
 */

#ifndef PRISMATIC_H
#define PRISMATIC_H

typedef struct {
    double J_Pri;    // Inertia
    double B_Pri;    // Damping
    double Eff_Pri;  // Efficiency
    double Ke_Pri;   // Back EMF constant (V·s/m)
    double Kt_Pri;   // Force constant (N/A)
    double R_Pri;    // Resistance
    double L_Pri;    // Inductance
} PrismaticMotor;

// Constructor
PrismaticMotor create_prismatic_motor(double J, double B, double Eff, double Ke, double Kt, double R, double L);

// Feedforward
float disturbance_feedforward_pri(const PrismaticMotor* motor, float load_force);
float reference_feedforward_pri(const PrismaticMotor* motor, float linear_velocity);

#endif // PRISMATIC_H

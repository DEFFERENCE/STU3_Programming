#ifndef REVOLUTE_H
#define REVOLUTE_H

typedef struct {
    double J_Rev;    // Moment of inertia
    double B_Rev;    // Damping coefficient
    double Eff_Rev;  // Efficiency
    double Ke_Rev;   // Back EMF constant
    double Kt_Rev;   // Torque constant
    double R_Rev;    // Resistance
    double L_Rev;    // Inductance
} RevoluteMotor;

// Constructor
RevoluteMotor create_motor(double J, double B, double Eff, double Ke, double Kt, double R, double L);


// Feedforward Control
float disturbance_feedforward(const RevoluteMotor* motor, double load_torque);
float reference_feedforward(const RevoluteMotor* motor, double angular_velocity);

#endif // REVOLUTE_H

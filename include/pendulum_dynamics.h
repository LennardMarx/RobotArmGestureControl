#ifndef PENDULUM_DYNAMICS_H
#define PENDULUM_DYNAMICS_H

#pragma once
#include "../include/pendulum.h"
#include "../include/helper_variables.h"
#include <array>
#include <vector>

class PendulumDynamics
{
public:
    PendulumDynamics();		  // contructor
    virtual ~PendulumDynamics(); // destructor

    // get and set methods
    bool getReset();
    void setReset(bool);

    std::array<double, 2> getReceivedInputs();
    void setReceivedInputs(std::array<double, 2>);

    std::array<double, 4> getReceivedStates();
    void setReceivedStates(const std::array<double, 4>&);

    std::array<double, 4> getUpdatedStates();
    void setUpdatedStates(std::array<double, 4>);

    bool getControllerState();
    void setControllerState(bool);

    std::array<double, 2> inverseKinematics(std::array<double, 2>);

    void toggleController();

    // virtual runge kutta methods to be overridden
    void rungeKutta();
    virtual std::array<double, 4> f(std::array<double, 4>, std::array<double, 2>&);
    std::array<double, 4> addArrays(std::array<double, 4>, std::array<double, 4>, double);

    std::array<int, 2> getRotations();
    void updateRotations();

    void print();

private:	// maybe change to private -> pass by reference
    double h = 0.001; // Time step
    bool reset = false;
    std::array<int, 2> rotations = { 0, 0 };

    std::array<double, 4> receivedStates; // array to save received drone states
    std::array<double, 2> receivedInputs; // array to save reveiced inputs
    std::array<double, 4> updatedStates;  // calculated new drone states
    std::array<double, 4> K1, K2, K3, K4; // RK4 intermediate steps

    const double g = 9.81;	    // Gravity
    const double pi = 3.141592653589793238462643383279502884197;

    double tau1, tau2; // motor torques
    int tau1_limit = 150; // limits on torques (motor contraints)
    int tau2_limit = 80;
    int kp = 300, kd = 20; // controller gains

    // Parts of equations of motion
    double det_M, w1_C, w2_C, w1_G, w2_G, w1_D, w2_D, term1, term2;
    double x_prev; // for saving x coord of previous frame

    std::array<int, 2> rotation_reference = { 0,0 }; // to count the turns of the desired position
    bool controllerState = 0;
};

#endif
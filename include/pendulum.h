#ifndef PENDULUM_H
#define PENDULUM_H

#pragma once
#include <array>
#include "../include/pendulum_dynamics.h"

class Pendulum
{
public:
    Pendulum(double, double); // constructor
    virtual ~Pendulum();	   // destructor

    // get and set methods
    std::array<double, 4>& getStates();
    void setStates(std::array<double, 4>);

    bool getReset();
    void setReset(bool);

    std::array<int, 2> getRotations();
    void updateRotations();

    std::array<double, 4>& getPreviousStates();
    void keepBetweenZeroAndPi();

private:
    const double pi = 3.141592653589793238462643383279502884197;
    bool reset = false;
    std::array<double, 4> pendulumStates; // drone states
    std::array<double, 4> previousStates;
    std::array<int, 2> rotations = { 0, 0 };

public:
    // variables of the robot arm
    // static -> immutable?
    // inline -> would else have to be declared (?) in .ccp
    static inline const double beta1 = 0.02; // Damping on joint 1
    static inline const double beta2 = 0.02; // Damping on joint 2
    static inline const double m1 = 1;		// Mass Link 1
    static inline const double m2 = 1;		// Mass Link 2
    static inline const double l1 = 0.5;		// Length Link 1
    static inline const double l2 = 0.5;		// Length Link 2
};

#endif
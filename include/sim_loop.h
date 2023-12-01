#ifndef SIM_LOOP_H
#define SIM_LOOP_H

#pragma once
#include "../include/UI.h"
#include "../include/pendulum.h"
#include "../include/pendulum_dynamics.h"
#include "../include/event_checks.h"
#include "../include/helper_variables.h"

#include <SDL2/SDL.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <ratio>

#include <utility>
#include <string>
#include <vector>
#include <unistd.h>
#include <list>

class SimLoop
{
public:
    SimLoop();
    ~SimLoop();

    void run();
private:
    const int FPS = 120;                // set FPS
    const int frameDelay = 1000 / FPS; // delay according to FPS
    Uint32 frameStart;                 // keeps track of time (?)
    int frameTime;

    int window_width = 1200;
    int window_height = 800;
    UI ui{ window_width, window_height };

    Pendulum pendulum;
    PendulumDynamics pendulumDynamics;
    HelperVars helperVars;
    EventChecks eventChecks;
    Uint32 mouseState;

    double pi = 3.141592653589793238462643383279502884197;
    double x0 = 0, y0 = 0, x1, y1, x2, y2; // link positions
    double x2_prev, y2_prev; // previous endeffector position
    double l1 = 150, l2 = 150; // pendulum length on screen

    int x, y;
    double x_conv;
    double y_conv;

    double qd1 = 0; // desired angle 1
    double qd2 = 0; // desired angle 2

    double xd = 0; // desired x (when doing inverse kinematics) (!! in meters not pixels !!)
    double yd = -1; // desired y

    bool pause = false;
    bool controllerOff = true;

    int frameCount = 0;
};

#endif
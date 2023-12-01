#ifndef HELPER_VARS_H
#define HELPER_VARS_H

#pragma once
#include "../include/pendulum.h"
#include <vector>
#include <array>

class HelperVars
{
public:
    HelperVars();
    ~HelperVars();

    void toggleQuit();
    bool getQuit();

    void toggleReset();
    bool getReset();

    void toggleTrajOn();
    bool getTrajOn();

    std::vector<std::array<double, 2>>& getTrajectory();

    void toggleController();
    bool getControllerState();

    std::array<int, 2>& getRotations();
private:
    static inline bool quit = false;
    static inline bool reset = false;
    static inline bool trajOn = false;
    static inline std::vector<std::array<double, 2>> trajectory;
    static inline bool controllerState = 1; // 1 = controller on, 0 = off
    static inline std::array<int, 2> rotations = { 0, 0 };
};

#endif
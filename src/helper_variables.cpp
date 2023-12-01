#include "../include/helper_variables.h"

HelperVars::HelperVars() {}
HelperVars::~HelperVars() {}

void HelperVars::toggleQuit() { quit = !quit; }
bool HelperVars::getQuit() { return quit; }

void HelperVars::toggleReset() { reset = !reset; }
bool HelperVars::getReset() { return reset; }

void HelperVars::toggleTrajOn() { trajOn = !trajOn; };
bool HelperVars::getTrajOn() { return trajOn; }

std::vector<std::array<double, 2>>& HelperVars::getTrajectory() { return trajectory; }

void HelperVars::toggleController()
{
    controllerState = !controllerState;
}
bool HelperVars::getControllerState() { return controllerState; }

std::array<int, 2>& HelperVars::getRotations() { return rotations; }
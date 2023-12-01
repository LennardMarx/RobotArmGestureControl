#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <cmath>

#include "../include/pendulum.h"

using namespace std::chrono_literals;

Pendulum::Pendulum(double _th1, double _th2): pendulumStates({ _th1, _th2, 0, 0 }) {}// constructor
Pendulum::~Pendulum() {} // destructor

std::array<double, 4>& Pendulum::getStates() { return pendulumStates; }
void Pendulum::setStates(std::array<double, 4> _x) { pendulumStates = _x; }

bool Pendulum::getReset() { return reset; }
void Pendulum::setReset(bool _reset) { reset = _reset; }

std::array<int, 2> Pendulum::getRotations()
{
    updateRotations();
    return rotations;
}

void Pendulum::updateRotations()
{
    rotations[0] = getStates()[0] / (2 * pi);
    rotations[1] = getStates()[1] / (2 * pi);
}

std::array<double, 4>& Pendulum::getPreviousStates() { return previousStates; }
void Pendulum::keepBetweenZeroAndPi()
{
    if (getStates()[0] >= 2 * pi && getPreviousStates()[0] < 2 * pi) { getStates()[0] -= 2 * pi; }
    if (getStates()[0] < 0 && getPreviousStates()[0] >= 0) { getStates()[0] += 2 * pi; }
}
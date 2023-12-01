#ifndef HAND_GESTURE_H
#define HAND_GESTURE_H

#include <Python.h>
#include <array>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

class HandGestures {
public:
  HandGestures();
  ~HandGestures();

  void init();
  void cleanUp();

  std::array<std::array<int, 2>, 21> estimateLandmarks();
  int fingerDistance(std::array<std::array<int, 2>, 21>);

private:
  FILE *fp;
  PyObject *pModule;
  PyObject *pFunc_run;
  PyObject *pFunc_distance;
  std::array<std::array<int, 2>, 21> landmarks;
};

#endif

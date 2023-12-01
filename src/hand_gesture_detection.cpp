#include "../include/hand_gesture_detection.h"

HandGestures::HandGestures() { init(); }
HandGestures::~HandGestures() { cleanUp(); }

int HandGestures::fingerDistance(std::array<std::array<int, 2>, 21> lm) {
  double dx = lm[8][0] - lm[4][0];
  double dy = lm[8][1] - lm[4][1];
  return int(std::sqrt(dx * dx + dy * dy));
}

void HandGestures::init() {
  // Initialize the Python interpreter
  Py_Initialize();

  fp = fopen("hand_gesture_recognition.py", "r");
  pModule = PyImport_ImportModule("__main__");
  PyModule_AddObject(pModule, "__builtins__", PyEval_GetBuiltins());
  PyRun_SimpleFile(fp, "hand_gesture_recognition.py");
  pFunc_run = PyObject_GetAttrString(pModule, "run");
  // pFunc_distance = PyObject_GetAttrString(pModule, "fingerDistance");
}

std::array<std::array<int, 2>, 21> HandGestures::estimateLandmarks() {
  PyObject *pValue = PyObject_CallObject(pFunc_run, NULL);
  PyObject *pList = PyList_AsTuple(pValue);
  Py_DECREF(pValue);
  // Extract the x and y coordinates from the tuple and print them
  for (int i = 0; i < 21; i++) {
    PyObject *pItem = PyTuple_GetItem(pList, i);
    double x = PyFloat_AsDouble(PyTuple_GetItem(PyList_AsTuple(pItem), 0));
    double y = PyFloat_AsDouble(PyTuple_GetItem(PyList_AsTuple(pItem), 1));
    // Py_DECREF(pItem);
    // std::cout << "Coordinate " << i << ": (" << x << ", " << y << ")" <<
    // std::endl;
    landmarks[i][0] = (int)(x / 400 * 1200 - 600);
    landmarks[i][1] = (int)(y / 400 * 800 - 400);
  }
  return landmarks;
}

void HandGestures::cleanUp() {
  Py_DECREF(pFunc_run);
  Py_DECREF(pModule);
  fclose(fp);
  Py_Finalize();
}

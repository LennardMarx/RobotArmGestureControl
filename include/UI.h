#ifndef UI_H
#define UI_H

#pragma once
#include <SDL2/SDL.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
// #include <SDL2/SDL_image.h>
#include <vector>

using namespace std::chrono_literals;

class UI {
public:
  const int sizeX;
  const int sizeY;

  UI(int, int);
  ~UI();

  void clear();
  void present();
  void drawPixel(int, int);
  void drawLine(int, int, int, int);
  void drawLine(int, int, int, int, int);
  void drawTiltedRectangle(double, double, double, double, double, int);
  void drawTrajectory(std::vector<std::array<double, 2>> &, int);
  void drawHandLandmarks(std::array<std::array<int, 2>, 21>);

  void setDrawColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a);
  SDL_Renderer *&getRenderer(); // pointer reference to the renderer
  SDL_Window *getWindow();      // pointer to the window

private:
  void initialize(int, int);

private:
  SDL_Window *window = nullptr;     // create window pointer
  SDL_Renderer *renderer = nullptr; // create renderer pointer
  bool quit;
};

#endif

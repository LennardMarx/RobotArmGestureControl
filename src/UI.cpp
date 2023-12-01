#include <SDL2/SDL.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
// #include <SDL2/SDL_image.h>
#include <iostream>
#include <vector>

#include "../include/UI.h"

using namespace std::chrono_literals;

UI::UI(int sizeX, int sizeY) : sizeX(sizeX), sizeY(sizeY) {
  initialize(sizeX, sizeY);
}

UI::~UI() {
  if (renderer)
    SDL_DestroyRenderer(renderer);
  if (window)
    SDL_DestroyWindow(window);
  SDL_Quit();
  // IMG_Quit();
  // SDL_DestroyTexture(_drone_texture);
}

void UI::clear() {
  setDrawColor(0, 70, 110, 255);
  SDL_RenderClear(renderer);
  setDrawColor(255, 255, 255, 255);
}

void UI::present() {
  // SDL_Delay(10);
  SDL_RenderPresent(renderer);
}
// function to draw pixel on screen (not used)
void UI::drawPixel(int x, int y) {
  SDL_RenderDrawPoint(renderer, x + sizeX / 2, y + sizeY / 2);
}
// function to draw line between two points
void UI::drawLine(int _x2, int _y2, int x2, int y2) {
  SDL_RenderDrawLine(renderer, _x2 + sizeX / 2, _y2 + sizeY / 2, x2 + sizeX / 2,
                     y2 + sizeY / 2);
}

void UI::drawLine(int _x2, int _y2, int x2, int y2, int thickness) {
  SDL_RenderDrawLine(renderer, _x2 + sizeX / 2, _y2 + sizeY / 2, x2 + sizeX / 2,
                     y2 + sizeY / 2);
}

// drawing a rectangle along a line (for links)
void UI::drawTiltedRectangle(double _x1, double _y1, double _x2, double _y2,
                             double _ang, int _width) {
  this->drawLine(_x1 + _width * sin(_ang), _y1 - _width * cos(_ang),
                 _x2 + _width * sin(_ang), _y2 - _width * cos(_ang));
  this->drawLine(_x1 - _width * sin(_ang), _y1 + _width * cos(_ang),
                 _x2 - _width * sin(_ang), _y2 + _width * cos(_ang));

  this->drawLine(_x1 + _width * sin(_ang), _y1 - _width * cos(_ang),
                 _x1 - _width * sin(_ang), _y1 + _width * cos(_ang));
  this->drawLine(_x2 + _width * sin(_ang), _y2 - _width * cos(_ang),
                 _x2 - _width * sin(_ang), _y2 + _width * cos(_ang));
}

// draw trajectory (intensity dependend on recency)
void UI::drawTrajectory(std::vector<std::array<double, 2>> &_trajectory,
                        int _length) {
  for (int i = 1; i < _trajectory.size(); i++) {
    this->setDrawColor(255, 255, 255, i);
    this->drawLine(_trajectory[i - 1][0], _trajectory[i - 1][1],
                   _trajectory[i][0], _trajectory[i][1]);
  }
  // only draw last X positions
  if (_trajectory.size() > _length) {
    _trajectory.erase(_trajectory.begin());
  }
}

void UI::drawHandLandmarks(std::array<std::array<int, 2>, 21> lm) {
  // SDL_RenderSetScale(renderer, .5, .5);
  drawLine(lm[0][0], lm[0][1], lm[1][0], lm[1][1]);
  drawLine(lm[1][0], lm[1][1], lm[2][0], lm[2][1]);
  drawLine(lm[2][0], lm[2][1], lm[3][0], lm[3][1]);
  drawLine(lm[3][0], lm[3][1], lm[4][0], lm[4][1]);

  drawLine(lm[0][0], lm[0][1], lm[5][0], lm[5][1]);
  drawLine(lm[5][0], lm[5][1], lm[6][0], lm[6][1]);
  drawLine(lm[6][0], lm[6][1], lm[7][0], lm[7][1]);
  drawLine(lm[7][0], lm[7][1], lm[8][0], lm[8][1]);

  drawLine(lm[5][0], lm[5][1], lm[9][0], lm[9][1]);
  drawLine(lm[9][0], lm[9][1], lm[10][0], lm[10][1]);
  drawLine(lm[10][0], lm[10][1], lm[11][0], lm[11][1]);
  drawLine(lm[11][0], lm[11][1], lm[12][0], lm[12][1]);

  drawLine(lm[9][0], lm[9][1], lm[13][0], lm[13][1]);
  drawLine(lm[13][0], lm[13][1], lm[14][0], lm[14][1]);
  drawLine(lm[14][0], lm[14][1], lm[15][0], lm[15][1]);
  drawLine(lm[15][0], lm[15][1], lm[16][0], lm[16][1]);

  drawLine(lm[13][0], lm[13][1], lm[17][0], lm[17][1]);

  drawLine(lm[0][0], lm[0][1], lm[17][0], lm[17][1]);
  drawLine(lm[17][0], lm[17][1], lm[18][0], lm[18][1]);
  drawLine(lm[18][0], lm[18][1], lm[19][0], lm[19][1]);
  drawLine(lm[19][0], lm[19][1], lm[20][0], lm[20][1]);
  // SDL_RenderSetScale(renderer, 1, 1);
}

void UI::setDrawColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
  SDL_SetRenderDrawColor(renderer, r, g, b, a);
}

SDL_Renderer *&UI::getRenderer() // pointer reference to the renderer
{
  return renderer;
}
SDL_Window *UI::getWindow() // pointer to the window
{
  return window;
}

// initializing the UI
void UI::initialize(int sizeX, int sizeY) {
  SDL_Init(SDL_INIT_EVERYTHING);

  // Create a Window
  window = SDL_CreateWindow("Robot Arm Gesture Control", 0, 0, sizeX, sizeY,
                            SDL_WINDOW_SHOWN);

  renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

  SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
  // scale window
  SDL_SetWindowSize(window, sizeX, sizeY);
  // adjust render scale
  SDL_RenderSetScale(renderer, 1, 1);
  // place window in middle of screen after scaling
  SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
}

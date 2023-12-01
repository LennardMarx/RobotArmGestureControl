#!/bin/bash

# g++ src/*.cpp -std=c++17 -I/usr/include/python3.10 -lpython3.10 -I/usr/include/SDL2/ -I. -Iinclude/ -lSDL2main -lSDL2 -lSDL2_ttf -o bin/robotarm
g++ src/*.cpp -std=c++17 -I/usr/include/python3.11 -lpython3.11 -I/usr/include/SDL2/ -I. -Iinclude/ -lSDL2main -lSDL2 -lSDL2_ttf -o bin/robotarm

#g++ -g -std=c++17 ./src/*.cpp -I./include -o ./bin/raycasting -lSDL2 -ldl

#!/bin/bash
echo "Jesus is Lord !!"

# clang++-22          # compiler (clang++ version 22)
# -std=c++20          # use C++20 standard
# -O2                 # optimization level 2 (faster code)
# -Wall               # show all warnings
# -o ./build/ctrl.o   # output object file destination
# -c ./code/ctrl.cpp  # compile only (no linking), input file

# In Case i ever want to use a make file
# build/ctrl: build/ctrl.o
# 	clang++-22 ./build/ctrl.o -o ./build/ctrl
# 
# build/ctrl.o: code/ctrl.cpp
# 	clang++-22 -std=c++20 -O2 -Wall -o ./build/ctrl.o -c ./code/ctrl.cpp

# Build Binary File
clang++-22 -std=c++20 -O2 -Wall  -o ./build/ctrl.o -c ./code/ctrl.cpp

# Link to executable Projekt
clang++-22 ./build/ctrl.o -o ./build/ctrl

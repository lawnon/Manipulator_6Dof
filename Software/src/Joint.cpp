/*
 * Program: Joint.cpp
 * Description: class object with wrapper fucktions
 *               for Servo.h class object
 * Autor: Chukwunonso Bob-Anyeji
 * Date: 09.06.2024
 */

#include "Joint.hpp"

Joint::Joint() {}

void Joint::Init(RobFrame* robFrame){
  _robFrame = robFrame;
}

void Joint::write(int degree) {
  int deg = degree + Joint::offset;

  if (deg < Joint::min) {
    Joint::servo.write(Joint::min);
  } else if (deg > Joint::max) {
    Joint::servo.write(Joint::max);
  } else {
    Joint::servo.write(deg);
  }
}

void Joint::attach(int pin) {
  Joint::servo.attach(pin);
}

void Joint::attach(int pin, int min, int max) {
  Joint::min = min;
  Joint::max = max;

  Joint::servo.attach(pin);
}

void Joint::attach(int pin, int min, int max, int offset) {
  Joint::min = min;
  Joint::max = max;
  Joint::offset = offset;

  Joint::servo.attach(pin);
}

void Joint::limit(int min, int max) {
  Joint::min = min;
  Joint::max = max;
}

int Joint::read() {
  int input = Joint::servo.read();

  if(input >= 180 || input <= 0 ){
    return input - Joint::offset;
  }
  else{
    return Joint::servo.read() - Joint::offset;
  }
}

int Joint::read2(){
  return Joint::servo.read();
}

int Joint::readMicroseconds() {
  return Joint::servo.readMicroseconds();
}

int Joint::delta(int dest){
  if (dest < Joint::min - Joint::offset){
    return Joint::min - Joint::offset;
  }
  if (dest > Joint::max - Joint::offset){
    return Joint::max - Joint::offset;
  }
  return dest - Joint::read();
}

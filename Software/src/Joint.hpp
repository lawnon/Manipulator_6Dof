/*
 * Program: Joint.hpp
 * Description: header file with wrapper fucktions
 *               for Servo.h class object
 * Autor: Chukwunonso Bob-Anyeji
 * Date: 09.06.2024
 */

#ifndef JOINT_H_
#define JOINT_H_

#include <Servo.h>

class RobFrame;
class Joint
{
private:
  int _offset;
  RobFrame* _robFrame;
public:
  Joint();

  void Init(RobFrame* robFrame);

  void write(int degree);
  void attach(int pin);
  void attach(int pin, int min, int max);
  void attach(int pin, int min, int max, int offset);
  void limit(int min, int max);
  int read();
  int read2();
  int readMicroseconds();
  int delta(int dest);

  int id = 0;
  int offset = 0;
  int min = 1;
  int max = 180;
  Servo servo;
};

#endif // JOINT_H_

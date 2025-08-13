#ifndef ROBFRAMEWORK_H_
#define ROBFRAMEWORK_H_

#include "pins.h"
#include "Joint.hpp"
#include "Commands.hpp"
#include "Kinematic.hpp"
#include "PositionFactory.hpp"

class RobFrame
{
private:
  int _loopCount = 0;

  void Increment(Joint& jt, int origin, int dest, int itr);
  void Sweep(Joint& jt, int origin, int dest);
  void Validate(Position& origin, Position& dest, int& range, int& valid, PosState state = PosState::PosAndPost);
  Position Interpolate(Position& origin, Position& dest, int range, int step = 1);
public:
  RobFrame();

  Posture PtHome {0, 135, 90, -85};

  Commands Commands;
  Kinematic Kinematics;
  PositionFactory PosFactory;

  Joint Joint1;
  Joint Joint2;
  Joint Joint3;
  Joint Joint4;
  Joint Joint5;
  Joint Joint6;

  void Setup();
  void Decode(String data);
  void GotoDeg(Joint& jt, int dest);
  void LoopPositions();

  PosState Drive(Posture pt);
  PosState Drive(Position ps);
  PosState LDrive(Position ps);
};

#endif // ROBFRAMEWORK_H_

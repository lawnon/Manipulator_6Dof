/*
 * Program: Commands.hpp
 * Description: header file containing prcedures
 *               for Parsing and validating Commands
 * Autor: Chukwunonso Bob-Anyeji
 * Date: 09.06.2024
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#define COMMAND_LEN 27

#include <Servo.h>
#include <wiring_private.h>

#include "Types.hpp"
#include "Logger.hpp"
#include "Joint.hpp"
#include "Kinematic.hpp"

class RobFrame;
class Commands
{
private:
  int _range = 0;
  RobFrame* _robFrame;
  Command _list[COMMAND_LEN];

public:
  Commands();

  // Const Difinitions
  enum Tags{
    /*0 */  World,
    /*  */  // Read Set Joint Positions
    /*1 */  Jt1,
    /*2 */  Jt2,
    /*3 */  Jt3,
    /*4 */  Jt4,
    /*5 */  Jt5,
    /*6 */  Jt6,
    /*7 */  Jt7,
    /*  */  // Get Rob Position Data
    /*8 */  Here,
    /*9 */  Joints,
    /*  */  // Drive to  Given Configurations
    /*10*/  GoHome,
    /*11*/  Drive,
    /*12*/  JDrive,
    /*13*/  LDrive,
    /*  */  // Read Set Parameters
    /*14*/  Speed,
    /*15*/  Accuracy,
    /*16*/  Debug,
    /*17*/  Voltage,
    /*  */  // Assignment and listing of Position Data
    /*18*/  Position,
    /*19*/  Posture,
    /*  */  // Listing of Parameter and Data
    /*20*/  ListPositions,
    /*21*/  ListPostures,
    /*22*/  ListAllPositions,
    /*  */  // Loops
    /*23*/  JLoop,
    /*24*/  LLoop,
    /*  */  // Teach / Manual Functions
    /*25*/  Rotate,
    /*26*/  RotStep,
    /*  */  // List and Document Command Parameters
    /*27*/  Help
  };

  Command& operator[](int index);

  Command Parse(String& input);
  Command Get(Tags tag);

  void Init(RobFrame* robFrame);

  int  GetParam(int cIndex);
  void SetParam(int cIndex, float val);
  void Delay();
  void List();
};

#endif // COMMANDS_H_

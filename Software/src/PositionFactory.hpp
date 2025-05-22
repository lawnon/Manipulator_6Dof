/*
 * Programm: PositionFactory.hpp
 * Description: header file containing class definitions
 *              for der creation, destruction and retrival and
 *              editing of position variables.
 * Autor: Chukwunonso Bob-Anyeji
 * Date: 11.07.2024
 */

#ifndef POSITIONFACTORY_H_
#define POSITIONFACTORY_H_

#define POS_LIMIT 100

#include "Types.hpp"
#include "Logger.hpp"

class RobFrame;
class PositionFactory
{
private:
  int8_t RangeCheck();
  sPosition TosPosition(Position ps);
  sPosture TosPosture(Posture ps);
  PosState Decode(String content, char delimiter = ',');
  void Update(int index,String& params, PosState state);

  PosData* _data;
  RobFrame* _robFrame;
  int _definedCount = 0;
public:
  PositionFactory();
  ~PositionFactory();

  void Init(RobFrame* robFrame);
  void Setup();
  Posture DecodePosture(String input, String filter = ",");
  Position DecodePosition(String input, String filter = ",");
  int& AddPosition(String content, PosState state, char filter = ' ');
  int& Count();
  void Get(String identifier);
  void Del(String identifier);
  void LogInfo();
  void LogPositions(PosState state);
  Position operator[](int index);
};

#endif // POSITIONFACTORY_H_

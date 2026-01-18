/*
 * Program: Commands.cpp
 * Description: Class-Object containing prcedures
 *               for Parsing and validating Commands
 * Autor: Chukwunonso Bob-Anyeji
 * Date: 09.06.2024
 */
#include "RobFrameWork.hpp"
#include "Commands.hpp"

using namespace Logger;

/* Command Functions */
CommandFactory::CommandFactory()
{
  _list[CommandFactory::World] = {CommandFactory::World, "world"};
  _list[CommandFactory::Jt1] = {CommandFactory::Jt1, "jt1"};
  _list[CommandFactory::Jt2] = {CommandFactory::Jt2, "jt2"};
  _list[CommandFactory::Jt3] = {CommandFactory::Jt3, "jt3"};
  _list[CommandFactory::Jt4] = {CommandFactory::Jt4, "jt4"};
  _list[CommandFactory::Jt5] = {CommandFactory::Jt5, "jt5"};
  _list[CommandFactory::Jt6] = {CommandFactory::Jt6, "jt6"};
  _list[CommandFactory::Jt7] = {CommandFactory::Jt7, "jt7"};
  _list[CommandFactory::Here] = {CommandFactory::Here, "here"};
  _list[CommandFactory::Joints] = {CommandFactory::Joints, "joints"};
  _list[CommandFactory::GoHome] = {CommandFactory::GoHome, "home"};
  _list[CommandFactory::Drive] = {CommandFactory::Drive, "drive"};
  _list[CommandFactory::JDrive] = {CommandFactory::JDrive, "jdrive"};
  _list[CommandFactory::LDrive] = {CommandFactory::LDrive, "ldrive"};
  _list[CommandFactory::Speed] = {CommandFactory::Speed, "speed", 30}; // in Percentage
  _list[CommandFactory::Accuracy] = {CommandFactory::Accuracy, "accuracy", 5};
  _list[CommandFactory::Debug] = {CommandFactory::Debug, "debug", -1};
  _list[CommandFactory::Voltage] = {CommandFactory::Voltage, "voltage", -1};
  _list[CommandFactory::Position] = {CommandFactory::Position, "position"};
  _list[CommandFactory::Posture] = {CommandFactory::Posture, "posture"};
  _list[CommandFactory::ListPositions] = {CommandFactory::ListPositions, "list-ps"};
  _list[CommandFactory::ListPostures] = {CommandFactory::ListPostures, "list-pt"};
  _list[CommandFactory::ListAllPositions] = {CommandFactory::ListAllPositions, "list-pspt"};
  _list[CommandFactory::JLoop] = {CommandFactory::JLoop, "jloop", -1};
  _list[CommandFactory::LLoop] = {CommandFactory::LLoop, "lloop", -1};
  _list[CommandFactory::Rotate] = {CommandFactory::Rotate, "teach-jt", -1};
  _list[CommandFactory::RotStep] = {CommandFactory::RotStep, "teach-step", 10};
  _list[CommandFactory::Help] = {CommandFactory::Help, "help"};

  _range = sizeof(_list) / sizeof(_list[0]);
}

void CommandFactory::Init(RobFrame *robFrame)
{
  _robFrame = robFrame;
}

Command &CommandFactory::operator[](int index)
{
  return _list[index];
}

Command CommandFactory::Parse(String& input)
{
  if (input.length() <= 0){
    return _cmdNaN;
  }
  input.toLowerCase();
  log(input.c_str(), "CMD::Parse: Incoming ");

  _cmdIndex = input.indexOf(" ");
  if (_cmdIndex <= -1){
    _cmdName = input;
    _cmdValue = "NaN";
  }else{
    _cmdName = input.substring(0, _cmdIndex);
    _cmdValue = input.substring(_cmdIndex + 1, input.length());
  }

  log(_cmdValue.length(), "CMD::Parse: CommandValue lenght ");

  for (int itr = 0; itr < _range; itr++)
  {
    if (_list[itr].Name.equals(_cmdName))
    {
      if (_cmdValue.length() > 0)
      {
        _list[itr].Content = _cmdValue;
        _list[itr].Value = _cmdValue.toFloat();
      }
      log(&_list[itr]);
      return _list[itr];
    }
  }

  log("CMD::Parse: Command Validation failed");
  return _cmdNaN;
}

Command CommandFactory::Get(CommandFactory::Tags tag)
{
  return _list[tag];
}

void CommandFactory::SetParam(int cIndex, float val)
{
  if (val < 0)
  {
    log(val, "CMD::SetParam: Invalid Parameter Value: ");
    return;
  }

  if ((cIndex < 0) && (cIndex >= _range))
  {
    log(cIndex, "CMD::SetParam: invalid Param Index: ");
    return;
  }

  _list[cIndex].Value = val;
}

int CommandFactory::GetParam(int cIndex)
{
  if (cIndex >= _range || cIndex < 0)
  {
    log(cIndex, "CMD::GetParam: Invalid Command Index");
    return -1;
  }
  return _list[cIndex].Value;
}

void CommandFactory::Delay()
{
  delay(10 - _list[Speed].Value / 10);
}

void CommandFactory::List()
{
  log("CMD: Command List");
  for (int itr = 0; itr < _range; itr++)
  {
    log(_list[itr].Name.c_str());
  }
}

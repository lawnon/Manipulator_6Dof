/*
 * Program: Commands.cpp
 * Description: Class-Object containing prcedures
 *               for Parsing and validating Commands
 * Autor: Chukwunonso Bob-Anyeji
 * Date: 09.06.2024
 */
#include "RobFrameWork.hpp"
#include "Commands.hpp"

/* Command Functions */
Commands::Commands()
{
  _list[Commands::World] = {Commands::World, "world"};
  _list[Commands::Jt1] = {Commands::Jt1, "jt1"};
  _list[Commands::Jt2] = {Commands::Jt2, "jt2"};
  _list[Commands::Jt3] = {Commands::Jt3, "jt3"};
  _list[Commands::Jt4] = {Commands::Jt4, "jt4"};
  _list[Commands::Jt5] = {Commands::Jt5, "jt5"};
  _list[Commands::Jt6] = {Commands::Jt6, "jt6"};
  _list[Commands::Jt7] = {Commands::Jt7, "jt7"};
  _list[Commands::Here] = {Commands::Here, "here"};
  _list[Commands::Joints] = {Commands::Joints, "joints"};
  _list[Commands::GoHome] = {Commands::GoHome, "home"};
  _list[Commands::Drive] = {Commands::Drive, "drive"};
  _list[Commands::JDrive] = {Commands::JDrive, "jdrive"};
  _list[Commands::LDrive] = {Commands::LDrive, "ldrive"};
  _list[Commands::Speed] = {Commands::Speed, "speed", 30}; // in Percentage
  _list[Commands::Accuracy] = {Commands::Accuracy, "accuracy", 5};
  _list[Commands::Debug] = {Commands::Debug, "debug", -1};
  _list[Commands::Voltage] = {Commands::Voltage, "voltage", -1};
  _list[Commands::Position] = {Commands::Position, "position"};
  _list[Commands::Posture] = {Commands::Posture, "posture"};
  _list[Commands::ListPositions] = {Commands::ListPositions, "list-ps"};
  _list[Commands::ListPostures] = {Commands::ListPostures, "list-pt"};
  _list[Commands::ListAllPositions] = {Commands::ListAllPositions, "list-pspt"};
  _list[Commands::JLoop] = {Commands::JLoop, "jloop", -1};
  _list[Commands::LLoop] = {Commands::LLoop, "lloop", -1};
  _list[Commands::Rotate] = {Commands::Rotate, "teach-jt", -1};
  _list[Commands::RotStep] = {Commands::RotStep, "teach-step", 10};
  _list[Commands::Help] = {Commands::Help, "help"};

  _range = sizeof(_list) / sizeof(_list[0]);
}

void Commands::Init(RobFrame *robFrame)
{
  _robFrame = robFrame;
}

Command &Commands::operator[](int index)
{
  return _list[index];
}

Command Commands::Parse(String &input)
{
  input.trim();
  input.toLowerCase();

  log(input, "CMD::Parse:[" + String(_range) + "] Incoming Data: ");

  int commandIndex = input.indexOf(" ");
  String commandName = input.substring(0, commandIndex);
  String commandValue = input.substring(commandIndex, input.length());
  commandValue.trim();

  log(commandValue.length(), "CMD::Parse: CommandValue lenght: ");

  for (int itr = 0; itr < _range; itr++)
  {
    if (_list[itr].Name.equals(commandName))
    {
      if (commandValue.length() > 0)
      {
        _list[itr].Content = commandValue;
        _list[itr].Value = commandValue.toFloat();
      }
      log(_list[itr]);
      return _list[itr];
    }
  }

  log("CMD::Parse: Command Validation failed");
  return {-1, "Invalid", 0};
}

Command Commands::Get(Commands::Tags tag)
{
  return _list[tag];
}

void Commands::SetParam(int cIndex, float val)
{
  if (val < 0)
  {
    log(val, "CMD::SetParam: Invalid Parameter Value: ");
    return;
  }

  if (cIndex < 0 & cIndex >= _range)
  {
    log(cIndex, "CMD::SetParam: invalid Param Index: ");
    return;
  }

  _list[cIndex].Value = val;
}

int Commands::GetParam(int cIndex)
{
  if (cIndex >= _range || cIndex < 0)
  {
    log(cIndex, "CMD::GetParam: Invalid Command Index");
    return -1;
  }
  return _list[cIndex].Value;
}

void Commands::Delay()
{
  delay(10 - _list[Commands::Speed].Value / 10);
}

void Commands::List()
{
  log("CMD: Command List");
  for (int itr = 0; itr < _range; itr++)
  {
    log(_list[itr].Name);
  }
}

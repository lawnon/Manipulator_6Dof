
//////////////////////////////////////////////////////////////////////////
// Datei: Actuator.cpp
// Beschreibung: Ansteuerungslogik des Aktuators
// Autor: Chukwunonso Bob-Anyeji
// Datum: 18.08.2025@12-00
// Aktualisiert: 27.08.2025@11-46
//////////////////////////////////////////////////////////////////////////

#include "RobFrameWork.hpp"
#include "Actuator.hpp"

using namespace Logger;
using namespace Utils;

/// Instanzierung des Aktuators bzw. Steppermotor Klassenobject
Actuator::Actuator(){}

/// RoboterFrameWork-Objekt Zeigerzuweisung
void Actuator::Init(RobFrame* robFrame){
  _robFrame = robFrame;
}

///
/// Private Funktions / Prozeduren
///

/// Winkelwerte in Grad zu Schritte konvertieren
int32 Actuator::calcSteps(int32 target)
{
  return (target*_motorSteps*_microSteps*_gearRatio)/360;
}

/// Schritte zu Winkel
int32 Actuator::calcPosition(int32 actSteps){
  return (actSteps*360)/(_motorSteps*_microSteps*_gearRatio);
}

///
/// Public Funktions / Prozeduren
///

/// Eindeutiges ID auslesen
int16 Actuator::Id(){
  return _id;
}

/// Hardware eigenschaften einrichten
int16 Actuator::SetUp(float gearRatio, int16 accuracy, int16 max, int16 min)
{
  _stepper->setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, 1000, 1000);
  _stepper->setEnableActiveState(LOW);
  _stepper->begin(_rpm,_microSteps);
  _stepper->disable();

  _actualSteps = 0;
  _previousSteps = 0;
  _gearRatio = int16(gearRatio);
  Accuracy = accuracy;
  _max = max;
  _min = min;

  _state = SetBit16(int16(_state), State::Stopped);
  _state = SetBit16(int16(_state), State::Referenced);
  _state = SetBit16(int16(_state), State::InPosition);

  return OK;
}

/// Anbindung der Hardwareausgänge am Klassen-Ojekt durch die Initialisierung
/// der "BasicStepperDriver" Klasse
int16 Actuator::Attach(int16 dirPin, int16 stepPin, int16 enaPin){
  _stepper = new BasicStepperDriver(_motorSteps, dirPin, stepPin, enaPin);
  _id = stepPin;

  return int16(_stepper);
}

/// Referenziehung der gespeicherten Ist-Position
int32 Actuator::Refernce(int32 value){
  _actualSteps = value;
  return _actualSteps;
}

/// Asynchrones Aktoransteuerung Einschalten bzw. Freigeben
int32 Actuator::Activate(int8 ena, uint16 tof)
{
  if(ena >= ON)
  {
    _stepper->enable();
    _state = SetBit16(_state, State::Enabled);
    _timeOfDelay = tof;
    //log(GetBit16(_state, State::InPosition), "Act:Inposition: ");
    //log(_targetSteps, "Act:target: ");
    //log(_actualSteps, "Act:Actual: ");
    if(GetBit16(_state, State::InPosition) <= OFF)
    {
      if(_stepper->nextAction() >= ON)
      {
        _state = SetBit16(_state, State::InMotion);
        _state = ResetBit16(_state, State::Stopped);

        _actualSteps = (_dir >= ON) ?
          _previousSteps + _stepper->getStepsCompleted():
          _previousSteps - _stepper->getStepsCompleted();
      }
      else
      {
        if (abs(_targetSteps -_actualSteps) <= Accuracy)
        {
          _state = SetBit16(_state, State::InPosition);
        }
      }
      return _stepper->nextAction();
    }
  }
  else
  {
    if(GetBit16(_state, State::Enabled) >= ON)
    {
      _timeOfDelay--;
      if(_timeOfDelay <= 0)
      {
        _stepper->disable();
        _state = ResetBit16(_state, State::Enabled);
        _state = ResetBit16(_state, State::InMotion);
        _state = SetBit16(_state, State::Stopped);
      }
      return ON;
    }
  }
  return OFF;
}

/// Sollpositon Synchrone Schreiben
int32 Actuator::Write(int32 target)
{
  if (target < _min)
  {
    target = _min;
  }

  if (target > _max)
  {
    target = _max;
  }
  int16 actpos = calcPosition(_actualSteps);
  if(target >= actpos)
  {
    target = target - actpos;
  }
  else
  {
    target = actpos - target;
  }

  _previousSteps = _actualSteps;
  _targetSteps = calcSteps(target);;

  // Delta ungleich Null Validierung
  if(_targetSteps == _actualSteps)
  {
    _state = SetBit16(_state, State::Aborted);
    return NOK;
  }

  // Fahrtrichtungs Validierung
  _dir = (_targetSteps >= _actualSteps) ? 1 : -1;
  _stepper->startMove(_targetSteps*_dir);
  log(_targetSteps, "Act:target: ");
  log(_actualSteps, "Act:Actual: ");
  log(_dir, "Act:Write dir ");
  log(_stepper->getDirection(), "Act:Write: Stepper Dir: ");
  if(_dir != _stepper->getDirection())
  {
    log(_id, "Act:Write Refernce Actor; ");
    _state = SetBit16(_state, State::Aborted);
    _state = ResetBit16(_state, State::Referenced);
    _stepper->startMove(0);
    return NOK;
  }

  log(_targetSteps, "Ziel Schritte: ");
  _state = ResetBit16(_state, State::InPosition);


  while (Activate(ON)>OK);
  return _targetSteps;
}

/// Ist-Winkelwert von in Grad auslesen
int32 Actuator::Read()
{
  return calcPosition(_actualSteps);
}

/// Soll-Ist Differenze bilden in Grad
int32 Actuator::Delta(int32 target){
  if (target < _min - _offset)
  {
    return _min - _offset;
  }
  if (target > _max - _offset)
  {
    return _max - _offset;
  }
  return target - Read();
}

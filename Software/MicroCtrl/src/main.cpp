// =======================================================
// File: Main.cpp
// Description: Applikations-Eingangspunkt
// Autor: Chukwunonso Bob-Anyeji
// Date: 19.08.202511-41
// =======================================================

// TODO: Refactor Pound defines
#define MOTOR_STEPS 200

#define JT1_STP_PIN 40
#define JT1_DIR_PIN 42
#define JT1_ENA_PIN 44

#define JT3_STP_PIN 60
#define JT3_DIR_PIN 61
#define JT3_ENA_PIN 56

/* Einbindung von Externen Code */
#include <math.h>
#include <BasicStepperDriver.h>

/* Einbindung von Internen Code */
#include "Types.hpp"
#include "Utilities.hpp"
#include "Actuator.hpp"
#include "Logger.hpp"

using namespace Logger;
using namespace Utils;

long CycleCount = 0;
int accu = 1;
int dir  = 1;
long step = -1;

//Actuator Actor1(MOTOR_STEPS, JT1_DIR_PIN, JT1_STP_PIN, JT3_ENA_PIN);
//Actuator Actor3(MOTOR_STEPS, JT3_DIR_PIN, JT3_STP_PIN, JT3_ENA_PIN);
Actuator Actor3();

void setup()
{
  // Initialisierungs Code Hier Einfügen
  Logger::logInit(115200);

  // Aktuator Initialisierung
  //Actor1.Init();
  //Actor3.Attach(JT3_DIR_PIN, JT3_STP_PIN, JT3_ENA_PIN);

  delay(3000);
  log("Hello World => SetUp done");
  log(sizeof(int8), "Integer Size");
  log(sizeof(int16), "Short Integger Size");
  log(sizeof(int32), "Long Integer Size");
  log(sizeof(int64), "Long long Integer Size");
  log("");
  int16 a = SetBit16(0,8);
  int16 as = GetBit16(a,8);
  int16 ass = ResetBit16(as,8);
  int16 asss = GetBit16(ass,a);
  log(a, "SetBit 8: ");
  log(as, "GetBit 8: ");
  log(ass, "ResetBit 8: ");
  log(asss, "GetBit 8: ");
  log("");
  log(int16(Actuator::State::Stopped), "Actuator State->First Item");
}

void loop()
{
  CycleCount++;

  if (Logger::Incoming() > 0 )
  {
    String str = Serial.readString();
    str.trim();
    log(CycleCount, str);

    step = str.toInt();
    log(CycleCount, "Motion Triggered: " + str);
  }

  if (step > 0 )
  {
    int target = min(accu, step);
    target = (dir/abs(dir)) * target;

    //Actor1.enable();
    //Actor3.enable();

    //Actor1.move(target);
    //Actor3.startMove(target);
    //Actor3.nextAction();

    //step = step - target;
  }

  if (step == 0)
  {
    //Actor1.disable();
    //Actor3.disable();

    delay(1000);
    log("Soll Position Erreict");
    step = -1;
  }
}

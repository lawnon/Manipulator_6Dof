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
#include <TinyMatrixMath.hpp>

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
Actuator Actor3;

void setup()
{
  // Initialisierungs Code Hier Einfügen
  Logger::logInit(115200);

  // Aktuator Initialisierung
  Actor3.Attach(JT3_DIR_PIN, JT3_STP_PIN, JT3_ENA_PIN);
  Actor3.SetUp(1,5,360,-360);

  delay(5000);
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

    // Creating a 3x3 matrix from a float array
  float A_raw[4][4] = {
  {1, 2, 3,4},
  {4, 5, 6,4},
  {9, 8, 9,1},
  {90, 80, 90,10},
  };

  tmm::Matrix<4,4> mat(A_raw);

  mat.printTo(Serial);
  log("------------ ");
  mat[3][2] = 0;
  mat.printTo(Serial);
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

  if (step != -1 )
  {
    int target = min(accu, step);
    target = (dir/abs(dir)) * target;

    Actor3.Write(step);
    step = -1;
    //Actor1.enable();
    //Actor3.enable();

    //Actor1.move(target);
    //Actor3.startMove(target);
    //Actor3.nextAction();

    //step = step - target;
    //log("Actor Position: " + String(Actor3.Read()));
  }
    Actor3.Activate(OK, 5000);

  if (step == 0)
  {
    //Actor1.disable();
    //Actor3.disable();

    delay(1000);
    log("Soll Position Erreict");
    step = -1;
  }
}

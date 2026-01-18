
//////////////////////////////////////////////////////////////////////////
// Datei: Main.cpp
// Beschreibung: Applikations-Eingangspunkt
// Autor: Chukwunonso Bob-Anyeji
// Datum: 19.08.202511-41
// Aktualisiert: 27.08.2025@11-46
//////////////////////////////////////////////////////////////////////////

#include "RobFrameWork.hpp"

using namespace Logger;
using namespace Utils;

long CycleCount = 0;
RobFrame RobArm;


// Load and Initialize Default Data
void setup()
{
  // Initialize Wait for Serial Monitor to Connect
  logInit(115200);

  // RobFramwork Initialization
  RobArm.Setup();
  log("Set Up Done");
}

// Execute Program Here
void loop()
{
  CycleCount++;

  // Get Inputs from Serial Port and Process
  if (Incoming() > 0)
  {
    // Read Input String
    String input = Serial.readString();
    input.trim();
    // Execute Commnand
    if (input.length() > 0){RobArm.Decode(input);}
  }

  /*Just for Testing*/
  //delay(10);
  if (false)
  {    
      if(CycleCount % 200 == 0)
      {
          log(CycleCount, "Hallo World");
      }
      if (CycleCount >= INT16_MAX)
      {
          log(CycleCount, "Max Cycle Count: ");
          CycleCount = 0;
      }
  }
}

/**/
///
/// Old Programm
///

/*

#include <BasicStepperDriver.h>
#include "RobFrameWork.hpp"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 30
#define MICROSTEPS 16

using namespace Logger;

long CycleCount = 0;
int RotPosition = 0;
int RotDirection = 0;
int RotDetected = 0;

RobFrame RobArm;

// Set Up Stepper Joints
BasicStepperDriver Jt1(MOTOR_STEPS, JT1_DIR_PIN, JT1_STP_PIN, JT1_ENA_PIN);
BasicStepperDriver Jt2(MOTOR_STEPS, JT2_DIR_PIN, JT2_STP_PIN, JT2_ENA_PIN);
BasicStepperDriver Jt3(MOTOR_STEPS, JT3_DIR_PIN, JT3_STP_PIN, JT3_ENA_PIN);
BasicStepperDriver Jt4(MOTOR_STEPS, JT4_DIR_PIN, JT4_STP_PIN, JT4_ENA_PIN);
BasicStepperDriver Jt5(MOTOR_STEPS, JT5_DIR_PIN, JT5_STP_PIN, JT5_ENA_PIN);
BasicStepperDriver Jt6(MOTOR_STEPS, JT6_DIR_PIN, JT6_STP_PIN, JT6_ENA_PIN);

void onEncoderRotation()
{
  //if (RobArm.Commands.GetParam(Commands::Rotate) > 0)
  {
    delay(4);
    if (digitalRead(ENC_CLK_PIN))
    {
      RotDirection = digitalRead(ENC_DIR_PIN);
    }
    else
    {
      RotDirection = !digitalRead(ENC_DIR_PIN);
    }
    RotDetected = 1;
    Serial.println("Rotation Detected");
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Wait for Serial Monitor to Connect
  while (!Serial)
  {
  }
  log("\n C++ Version: " + String(__cplusplus));

  //RobArm.Setup();

  // Set Uö Primitive Stepper
  Jt1.begin(5, MICROSTEPS);
  Jt2.begin(RPM, MICROSTEPS);
  Jt3.begin(RPM, MICROSTEPS);
  Jt4.begin(RPM, MICROSTEPS);
  Jt5.begin(RPM, MICROSTEPS);
  Jt6.begin(RPM, MICROSTEPS);

  Jt1.setEnableActiveState(LOW);
  Jt2.setEnableActiveState(LOW);
  Jt3.setEnableActiveState(LOW);
  Jt4.setEnableActiveState(LOW);
  Jt5.setEnableActiveState(LOW);
  Jt6.setEnableActiveState(LOW);

  Jt1.disable();
  Jt2.disable();
  Jt3.disable();
  Jt4.disable();
  Jt5.disable();
  Jt6.disable();

  // Set Up Rotary Encoder
  pinMode(ENC_CLK_PIN, INPUT);
  pinMode(ENC_DIR_PIN, INPUT);
  pinMode(ENC_BTN_PIN, INPUT);
  digitalWrite(ENC_BTN_PIN, HIGH);
  attachInterrupt(ENC_INTP_ID, onEncoderRotation, FALLING);

  log("Hello World => SetUp done");
}

void loop()
{
  CycleCount++;

  int enc_btn = !digitalRead(ENC_BTN_PIN);
  if (enc_btn >= 1)
  {
    Serial.println("ENC_BTN_PIN Pressed");
    delay(100);

    Jt4.enable();
    Jt4.rotate(180*10);
    delay(1000);
    Jt4.rotate(-180*10);
    delay(1000);
    Jt4.rotate(-180*10);
    delay(1000);
    Jt4.rotate(180*10);
    Jt4.disable();
  }

  // Get Inputs from Serial Port and Process
  if (Serial.available() > 0)
  {
    //RobArm.Decode(Serial.readString());
  }

  if (RotDetected >= 1)
  {
    int step = 0;
    if (RotDirection >= 1)
    {
      step = 1;
    }
    else
    {
      step = -1;
    }
    //Jt6.enable();
    Jt2.enable();
    Jt2.rotate(step * 32);
    //Jt6.rotate(-step * 8);
    delay(1);
    //Jt3.disable();
    //Jt6.disable();

    // switch (RobArm.Commands.GetParam(Commands::Rotate))
    // {
    // case Commands::Jt1:
    //   Jt1.enable();
    //   Jt1.rotate(step * 16);
    //   delay(1);
    //   break;
    // case Commands::Jt2:
    //   Jt2.enable();
    //   Jt2.rotate(step * 128);
    //   delay(1);
    //   break;
    // case Commands::Jt3:
    //   Jt3.enable();
    //   Jt3.rotate(step * 128);
    //   delay(1);
    //   break;
    // case Commands::Jt4:
    //   Jt4.enable();
    //   Jt4.rotate(step * 16) ;
    //   delay(1);
    //   break;
    // case Commands::Jt5:
    //   Jt5.enable();
    //   Jt5.rotate(step * 128);
    //   delay(1);
    //   Jt5.disable();
    //   break;
    // case Commands::Jt6:
    //   Jt6.enable();
    //   Jt6.rotate(step * 16);
    //   delay(1);
    //   Jt6.disable();
    //   break;
    // }
    //
    // RotDetected = -1;
  }
}
*/


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
  // Set Up Serial Monitor
  Serial.begin(9600);

  // Wait for Serial Monitor to Connect
  while (!Serial)
  {
  }
  delay(5000);
  log("\n C++ Version: " + String(__cplusplus));

  // RobFramwork Initialization
  RobArm.Setup();
  log("Set Up Done");
}

// Execute Program Here
void loop()
{
  CycleCount++;

  // Get Inputs from Serial Port and Process
  if (Serial.available() > 0)
  {
    log("INCOMING");
    RobArm.Decode(Serial.readString());
  }
}

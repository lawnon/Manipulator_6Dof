#include <BasicStepperDriver.h>

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 30
#define MICROSTEPS 16

#define JT3_STP_PIN 60
#define JT3_DIR_PIN 61
#define JT3_ENA_PIN 56

long CycleCount = 0;
int RotPosition = 0;
int RotDirection = 0;
int RotDetected = 0;
int step = 0;

// Set Up Stepper Joints
BasicStepperDriver Jt3(MOTOR_STEPS, JT3_DIR_PIN, JT3_STP_PIN, JT3_ENA_PIN);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Wait for Serial Monitor to Connect
  while (!Serial)
  {
  }
  Serial.println("\n C++ Version: " + String(__cplusplus));

  // Aktuator Initialization
  Jt3.begin(RPM, MICROSTEPS);
  Jt3.setEnableActiveState(HIGH);
  Jt3.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED,1000,1000);

  Jt3.disable();

  Serial.println("Hello World => SetUp done");
  delay(5000);
}

void loop()
{
  CycleCount++;
  Serial.println(String(CycleCount) + ": Start Async_Rotate");

  Jt3.rotate(step);
  delay(1000);
  Serial.println(
    "Async_Rotate Ist Steps: " + String(step));

  step++;
  //Serial.println(
  //  "Async_Rotate Delta Steps: " + String(Jt3.getStepsRemaining()));



  /*
  Serial.println(String(CycleCount) + ": Start Sync_Rotate 360");
  Jt3.rotate(360);
  Serial.println(String(CycleCount) + ": Steps =>" + String(Jt3.getSteps()));
  delay(1000);
  Serial.println(String(CycleCount) + ": Sync_Rotate -360");
  Jt3.rotate(-360);
  Serial.println(String(CycleCount) + ": Steps =>" + String(Jt3.getSteps()));
  delay(1000);
   */
}

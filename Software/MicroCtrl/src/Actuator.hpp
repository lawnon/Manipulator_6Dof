// ========================================================================
// File: Actuator.hpp
// Description: header file with wrapper functions with respect to Hardware
// Actuator
// Autor: Chukwunonso Bob-Anyeji
// Date: 09:06.2025
// ========================================================================

// NOTE: Motor steps per revolution.
// Most steppers are 200 steps or 1.8 degrees/step

#ifndef ACTUATOR_H_
#define ACTUATOR_H_

#include <BasicStepperDriver.h>
#include "Types.hpp"
#include "Utilities.hpp"
#include "Logger.hpp"

class Actuator
{
    private:
        // Zeiger zur Stepper Objekt
        BasicStepperDriver* _stepper;
        // Stepper und Steppertreiber Einstellung
        uint8 _motorSteps = 200;
        uint8 _microSteps = 16;
        // Konfigurationswerte
        int8  _dir = 1;
        int16 _id = 0;
        int16 _offset;
        int16 _min = -90;
        int16 _max = 90;
        int16 _gearRatio = 1;
        int32 _previousSteps = 0;
        int32 _actualSteps = 0;
        int32 _targetSteps = 0;
        uint16 _timeOfDelay = 0;
        float _rpm = 10;
        // Zustands Variabeln
        int8  _enabled = -1;
        word _state = 0;
        // Funktionen / Prozeduren
        int32 calcSteps(int32 target);
        int32 calcPosition(int32 actSteps);
    public:
        // Klassen Objektinstanz
        Actuator ();
        // Initialisierung
        int16 Id();
        int16 SetUp(float gearRatio = 1, int16 accuracy = 5, int16 max = 90, int16 min = -90);
        int16 Attach(int16 dirPin, int16 stepPin, int16 enaPin);
        // Attribute
        int16 Accuracy;
        // Betriebssteuerung
        int32 Activate(int8 ena, uint16 tof = 5000);
        int32 Refernce(int32 value = 0);
        // Steuerungs-Funktionen
        int32 Write(int32 target);
        int32 Read();
        int32 Delta(int32 target);
        // Enumerationen
        enum State {Enabled, Stopped, InMotion, Referenced, InPosition, Aborted};
};

#endif // ACTUATOR_H_

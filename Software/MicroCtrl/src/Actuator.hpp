
//////////////////////////////////////////////////////////////////////////
// Datei: Actuator.hpp
// Beschreibung: Ansteuerungslogik des Aktuators.
//
//               Dieser Klasse dient als wrappper für die
//               "BasicStepperDriver" Klasss von Laurentiu Badea
//               https://github.com/laurb9/StepperDriver
//
// Autor: Chukwunonso Bob-Anyeji
// Datum: 18.08.2025@12-00
// Aktualisiert: 27.08.2025@11-46
//////////////////////////////////////////////////////////////////////////

// NOTE: Motor steps per revolution.
// die meisten Nema-17 Stepper haben 200 steps oder 1.8 degrees/step
// mit 16 Miktrosteps Einstellung jenach Treiber

#ifndef ACTUATOR_H_
#define ACTUATOR_H_

#include <BasicStepperDriver.h>
#include "Types.hpp"
#include "Utilities.hpp"
#include "Logger.hpp"

class RobFrame;
class Actuator
{
    private:
        /// Zeiger zu RobFrame
        RobFrame* _robFrame;
        /// Zeiger zur Stepper Objekt
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
        /// @param target Winkel
        /// @return void
        int32 calcSteps(int32 target);
        /// @param Schritte
        /// @return Winkel
        int32 calcPosition(int32 actSteps);
    public:
        /// @brief Actor Klassen-Objekt
        Actuator ();
        /// @brief Initialisierung Zeiger-Zuweisung
        /// @param robFrame Zeiger zu RobFrame
        /// @return void
        void Init(RobFrame* robFrame);
        /// @brief Aktor ID
        int16 Id();
        /// @brief Hardwareeinrichten
        /// @param gearRatio Getriebeübersetzung
        /// @param accuracy Genauigkeit
        /// @param max Max-Winkelstellung
        /// @param min Min-Winkelstellung
        /// @return OK wenn Erfolgreich
        int16 SetUp(float gearRatio = 1, int16 accuracy = 5, int16 max = 90, int16 min = -90);
        /// @brief Hardware-Ausgänge Zuweisen
        /// Anbinddung der Hardware-Ausgänge
        /// @param dirPin Richtungs Pin id.
        /// @param stepPin Stepper Pin id.
        /// @param enaPin Aktivierungs Pin id.
        /// @return Zeiger-Id  zur BasicSteperDriver objekt
        int32 Attach(int16 dirPin, int16 stepPin, int16 enaPin);
        ///  Genauigkeit
        int16 Accuracy;
        /// @brief Aktoransteuerung
        /// @param ena Aktivieren
        /// @param tof Ausschaltverzögerung
        /// @return ON wenn activ oder OFF wenn nicht Active
        int32 Activate(int8 ena, uint16 tof = 5000);
        /// @brief IstPositionswert Referenzieren
        /// @param value Zurücksetzungswert
        /// @return neuer Istposition
        int32 Refernce(int32 value = 0);
        /// @brief Aktuator Sollwert Schreiben
        /// @param target Zielvorgabe in Grad
        /// @return Anzahl gefahrene Schritte.
        int32 Write(int32 target);
        /// @brief lese Istposition
        /// @return Istwert in Grad
        float Read();
        /// @brief Soll-Ist Differenze bilden
        /// @param target Zielposition
        /// @return Winkel differenze
        int32 Delta(int32 target);
        /// Aktuator Betriebszustand
        enum State {Enabled, Stopped, InMotion, Referenced, InPosition, Aborted};
};

#endif // ACTUATOR_H_


//////////////////////////////////////////////////////////////////////////
// File: RobFramWork.hpp
// Beschreibung: Klassen Objekt zur Verwaltung, Steuerung und ausführung
//               Des Manipulators
// Autor: Chukwunonso Bob-Anyeji
// Datum: 18.08.2025@12-00
// Aktualisiert: 27.08.2025@11-46
//////////////////////////////////////////////////////////////////////////

#ifndef ROBFRAMEWORK_H_
#define ROBFRAMEWORK_H_

#include "pins.h"
#include "Actuator.hpp"
#include "Commands.hpp"
#include "Kinematic.hpp"
#include "PositionFactory.hpp"

class RobFrame
{
  private:
    /// Schleifen zähler
    int _loopCount = 0;
    /// @brief Aktor Asynchron ansteuern
    /// @param jt Aktuator referenz
    /// @param origin Ist-Position
    /// @param dest Ziel-Position
    /// @param itr Inkrementen Wert
    /// @return void
    void Increment(Actuator& jt, int origin, int dest, int itr);
    /// @brief Aktor Synchron Ansteuern
    /// @param jt Aktuator referenz
    /// @param origin Ist-Postion
    /// @param dest Ziel-Position
    /// @return void
    void Sweep(Actuator& jt, int origin, int dest);
    /// @brief Positions Soll, Ist-Positions vergleich
    /// @param origin Ist-Position
    /// @param dest Soll-Position
    /// @param range Tolerance
    /// @param Flag Validierungs Merker 1 wenn Ist gleich Soll-Position,
    ///        0 wenn Ist ungleich Soll-Position.
    /// @param state Datensatz Type
    /// @return void
    void Validate(Position& origin, Position& dest, int& range, int& valid, PosState state = PosState::PosAndPost);
    /// Gradlinige Interpolation zwischen Zwei Punkte
    Position Interpolate(Position& origin, Position& dest, int range, int step = 1);
  public:
    /// @brief Klassen-Objekt
    RobFrame();
    /// @brief Grundstellung
    Posture PtHome {0, 45, -90, 0, -45, 0};
    /// @brief Befehls-Manager
    Commands Commands;
    /// @brief Kinematik-Manager
    Kinematic Kinematics;
    /// @brief Position-Manager
    PositionFactory PosFactory;
    /// @brief Gelenk 1
    Actuator Joint1;
    /// @brief Gelenk 2
    Actuator Joint2;
    /// @brief Gelenk 3
    Actuator Joint3;
    /// @brief Gelenk 4
    Actuator Joint4;
    /// @brief Gelenk 5
    Actuator Joint5;
    /// @brief Gelenk 6
    Actuator Joint6;
    /// @brief Einrichtung und Speicherallokation
    void Setup();
    /// @brief Eingangsdaten Parsen
    /// @param data String data
    /// @return void
    void Decode(String data);
    /// @brief Glenk Einzelverfahren
    /// @param jt Aktuator Referenz
    /// @param dest Ziel-Position
    /// @return void
    void GotoDeg(Actuator& jt, int dest);
    /// @brief Iteriere durch den Positionsdatensatz
    /// @return void
    void LoopPositions();
    /// @brief Punkt zu Punkt Asynchrone Gelenk-Ansteuerung
    /// @param pt Soll Winkelstellung
    /// @return Positions Status
    PosState Drive(Posture pt);
    /// @brief Punkt zu Punkt Asynchrone Gelenk-Ansteuerung
    /// @param ps Soll Position
    /// @return Positions Status
    PosState Drive(Position ps);
    /// @brief Gradlinige  Asynchrone Gelenk-Ansteuerung
    /// @param ps Soll Position
    /// @return Positions Status
    PosState LDrive(Position ps);
};

#endif // ROBFRAMEWORK_H_

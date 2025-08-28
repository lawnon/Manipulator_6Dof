
//////////////////////////////////////////////////////////////////////////
// File: RobFramWork.cpp
// Beschreibung: Klassen Objekt zur Verwaltung, Steuerung und ausführung
//               Des Manipulators
// Autor: Chukwunonso Bob-Anyeji
// Datum: 18.08.2025@12-00
// Aktualisiert: 27.08.2025@11-46
//////////////////////////////////////////////////////////////////////////

#include "RobFrameWork.hpp"

using namespace Logger;

RobFrame::RobFrame(){
  Kinematics.Init(this);
  Commands.Init(this);
  PosFactory.Init(this);

  Joint1.Init(this);
  Joint2.Init(this);
  Joint3.Init(this);
  Joint4.Init(this);
  Joint5.Init(this);
  Joint6.Init(this);
}

/// Einrichtung des Aktoren, und Positions Daten allozieren
void RobFrame::Setup(){
  // Attach Joint Parameters
  Joint1.Attach(JT1_DIR_PIN, JT1_STP_PIN, JT1_ENA_PIN);
  Joint2.Attach(JT2_DIR_PIN, JT2_STP_PIN, JT2_ENA_PIN);
  Joint3.Attach(JT3_DIR_PIN, JT3_STP_PIN, JT3_ENA_PIN);
  Joint4.Attach(JT4_DIR_PIN, JT4_STP_PIN, JT4_ENA_PIN);
  Joint5.Attach(JT5_DIR_PIN, JT5_STP_PIN, JT5_ENA_PIN);
  Joint6.Attach(JT6_DIR_PIN, JT6_STP_PIN, JT6_ENA_PIN);
  // Set Up Joints
  Joint1.SetUp( 1,    5,  90, -90);
  Joint2.SetUp(13.73, 5,  90, -90);
  Joint3.SetUp(10,    5,  90, -90);
  Joint4.SetUp( 1,    5,  90, -90);
  Joint5.SetUp(10,    5,  90, -90);
  Joint6.SetUp( 1,    5, 180, -180);
  // Set Up Initial Configurations
  Joint1.Write(PtHome.Jt1);
  Joint2.Write(PtHome.Jt2);
  Joint3.Write(PtHome.Jt3);
  Joint4.Write(PtHome.Jt4);
  Joint5.Write(PtHome.Jt5);
  Joint6.Write(PtHome.Jt6);
  // Allocation of Static Position Data
  PosFactory.Setup();
}

/// Verfahre Gelenk Einzel zum Soll-Position
void RobFrame::GotoDeg(Actuator& jt, int dest) {
  float origin = jt.Read();
  Sweep(jt, origin, dest);
}

/// Manipulator Aktorik synchron ansteuern d.h. mit
/// Thread Sperrung
void RobFrame::Sweep(Actuator& jt, int origin, int dest) {
  //log("Sweep Started");
  //log("Jt" + String(jt.id) + " at " + String(origin));
  //log("Jt" + String(jt.id) + " goto " + String(dest));

  if (dest >= origin){
    for (int i = origin; i <= dest; i += 1) {
      // in steps of 1 degree
      jt.Write(i);
      Commands.Delay();
    }
  }else{
    for (int i = origin; i >= dest; i -= 1) {
      jt.Write(i);
      Commands.Delay();
    }
  }
  //log("Sweep Done");
}

/// Gelenk Aktorik schrittweise inkrementieren asynchrone
/// Ansteuerung zur ermöglichen
void RobFrame::Increment(Actuator& jt, int origin, int dest, int itr){
  if (origin <= dest) {
    if (origin + itr <= dest) {
      jt.Write(origin + itr);
      //log(origin + itr, "Fwd JT" + String(jt.id));
    }
  }
  else{
    if (origin  - itr >= dest){
      jt.Write(origin - itr);
      //log(origin - itr, "Bwd JT" + String(jt.id));
    }
  }
}

/// Ist-Soll Positions Vergleich für die validierung der
/// Inverskinematik Berechnung
void RobFrame::Validate(Position& origin,
                        Position& dest,
                        int& range,
                        int& valid,
                        PosState state = PosState::PosAndPost){
  valid = (fabs(dest.X - origin.X) <= range);
  valid = valid && (fabs(dest.Y - origin.Y) <= range);
  valid = valid && (fabs(dest.Z - origin.Z) <= range);
  if(state != PosState::Position){
    valid = valid && (fabs(dest.B - origin.B) <= range);
  }
}

/// Gradlinige Interpolation zwischen zwei punkte
/// delta = Ist-Position + Schritt*(sign(Richtung))
Position RobFrame::Interpolate(Position& origin,
                               Position& dest,
                               int range,
                               int step = 1){
  return Position {
    ((dest.X - origin.X) > range) ? origin.X + step*((dest.X - origin.X)/abs(dest.X - origin.X)) : dest.X,
    ((dest.Y - origin.Y) > range) ? origin.Y + step*((dest.Y - origin.Y)/abs(dest.Y - origin.Y)) : dest.Y,
    ((dest.Z - origin.Z) > range) ? origin.Z + step*((dest.Z - origin.Z)/abs(dest.Z - origin.Z)) : dest.Z,
    ((dest.A - origin.A) > range) ? origin.A + step*((dest.A - origin.A)/abs(dest.A - origin.A)) : dest.A,
    ((dest.B - origin.B) > range) ? origin.B + step*((dest.B - origin.B)/abs(dest.B - origin.B)) : dest.B,
    ((dest.C - origin.C) > range) ? origin.C + step*((dest.C - origin.C)/abs(dest.C - origin.C)) : dest.C
  };
}

/// Verfahre alle Gelenke Asynchrone zum Ziel Stellung
/// in Punkt zu Punkt Bewegung
PosState RobFrame::Drive(Posture pt){
  log("Drive Data: " + String(pt.Jt1) +
      " | " + String(pt.Jt2) +
      " | " + String(pt.Jt3) +
      " | " + String(pt.Jt4) +
      " | " + String(pt.Jt5) +
      " | " + String(pt.Jt6));

  if(pt.Jt1 == 0 && pt.Jt2 == 0 && pt.Jt3 == 0
     && pt.Jt4 == 0 && pt.Jt5 == 0 && pt.Jt6 == 0){
    return;
  }

  int origin1 = Joint1.Read();
  int origin2 = Joint2.Read();
  int origin3 = Joint3.Read();
  int origin4 = Joint4.Read();
  int origin5 = Joint5.Read();
  int origin6 = Joint6.Read();

  int delta1 = Joint1.Delta(pt.Jt1);
  int delta2 = Joint2.Delta(pt.Jt2);
  int delta3 = Joint3.Delta(pt.Jt3);
  int delta4 = Joint4.Delta(pt.Jt4);
  int delta5 = Joint5.Delta(pt.Jt5);
  int delta6 = Joint6.Delta(pt.Jt6);
  log("Drive Delta: " + String(delta1) +
      " | " + String(delta2) +
      " | " + String(delta3) +
      " | " + String(delta4) +
      " | " + String(delta5) +
      " | " + String(delta6));

  // Get max delta
  int delta = (abs(delta1) >= abs(delta2)) ? abs(delta1) : abs(delta2);
  delta = (abs(delta) >= abs(delta3)) ? abs(delta) : abs(delta3);
  delta = (abs(delta) >= abs(delta4)) ? abs(delta) : abs(delta4);
  delta = (abs(delta) >= abs(delta5)) ? abs(delta) : abs(delta5);
  delta = (abs(delta) >= abs(delta6)) ? abs(delta) : abs(delta6);

  // Iterate Through delta Range and Increment
  for(int itr = 1; itr <= delta; itr++){
    //log(itr, "iterator");
    Increment(Joint1, origin1, pt.Jt1, itr);
    Increment(Joint2, origin2, pt.Jt2, itr);
    Increment(Joint3, origin3, pt.Jt3, itr);
    Increment(Joint4, origin4, pt.Jt4, itr);
    Increment(Joint5, origin5, pt.Jt5, itr);
    Increment(Joint6, origin6, pt.Jt6, itr);
    // Delay with respect to Speed
    Commands.Delay();
  }
  return PosState::Defined;
}

/// Verfahre alle Gelenke Asynchron zum Ziel Position
/// mit einer Point zu Punkt bzw. Gelenk Bewegung
PosState RobFrame::Drive(Position ps){
  Posture pt = Kinematics.ivKinematic(ps);
  Position psCheck = Kinematics.fdKinematic(pt);

  int range = 1;
  int valid = 0;

  Validate(ps, psCheck, range, valid);

  if(valid != 1){
    logln(ps,"Validation of target Position failed");
    logln(psCheck,"psChect");
    return PosState::Invalid;
  }
  return Drive(pt);
}

/// Verfahre alle Gelenke Asynchron zum Ziel Position
/// mit einer gradlinige  Bewegung
PosState RobFrame::LDrive(Position ps){
  int valid = 1;
  int counter = 0;
  PosState result = PosState::Undefined;
  int range = Commands[Commands::Accuracy].Value;
  Position origin {};

  do{
    origin = Kinematics.fdKinematic(Posture {
        Joint1.Read(),
        Joint2.Read(),
        Joint3.Read(),
        Joint4.Read(),
        Joint5.Read(),
        Joint6.Read()
      });

    logln(origin, "RF::LDrive: origin");

    Validate(origin, ps, range, valid, PosState::Position);
    log(valid,"RF:LDrive: Valid");
    if(valid <= 0){
      Position itp = Interpolate(origin, ps, range);
      if(Commands[Commands::Debug].Value > 0){
        logln(itp, "RF::LDrive: Intp");
      }
      result = Drive(itp);
    }

    counter++;
  }while(valid <= 0 && counter < 1000000 && result != PosState::Invalid);
  log(counter, "RF::LDrive: Counter ");

  return result;
}

/// Iterierer durch den positions Datensatz
/// Fahre in Punkt zu Punkt bzw. Joint Bewegung
/// oder fahre Gradlinige Bewegung.
void RobFrame::LoopPositions(){
  if(Commands[Commands::JLoop].Value > 0 &&
     Commands[Commands::LLoop].Value <= 0){
    if(_loopCount < PosFactory.Count()){
      logln(PosFactory[_loopCount], "RF::LoopPos: [" + String(_loopCount) + "]");
      Drive(PosFactory[_loopCount]);
      _loopCount++;
      Commands.Delay();
    }else{
      _loopCount = 0;
    }
  }
  if(Commands[Commands::LLoop].Value > 0 &&
     Commands[Commands::JLoop].Value <= 0){
    if(_loopCount < PosFactory.Count()){
      logln(PosFactory[_loopCount], "RF::LoopPos: [" + String(_loopCount) + "]");
      LDrive(PosFactory[_loopCount]);
      _loopCount++;
      Commands.Delay();
    }else{
      _loopCount = 0;
    }
  }
}

/// Eingangs Daten aus Serialle Port lesen
/// Auswerten und entsprechend aus führen.
void RobFrame::Decode(String data) {
  // Eingangs Daten lesen
  Command inputCmd = Commands.Parse(data);

  switch (inputCmd.Id) {
  case Commands::World:
    log("World Command not yet Implemented");
    break;

  case Commands::Jt1:
    GotoDeg(Joint1, inputCmd.Value);
    break;

  case Commands::Jt2:
    GotoDeg(Joint2, inputCmd.Value);
    break;

  case Commands::Jt3:
    GotoDeg(Joint3, inputCmd.Value);
    break;

  case Commands::Jt4:
    GotoDeg(Joint4, inputCmd.Value);
    break;

  case Commands::Joints:
    // Get Angular Positoin of each Joint
    log(Posture {
        Joint1.Read(),
        Joint2.Read(),
        Joint3.Read(),
        Joint4.Read(),
        Joint5.Read(),
        Joint6.Read()
      });
    break;

  case Commands::Here:
    // Get Carthesian Koordinates
    log(Kinematics.fdKinematic(Posture {
          Joint1.Read(),
          Joint2.Read(),
          Joint3.Read(),
          Joint4.Read(),
          Joint5.Read(),
          Joint6.Read()
        }));
    break;

  case Commands::GoHome:
    Drive(Posture {PtHome.Jt1, PtHome.Jt2, PtHome.Jt3, PtHome.Jt4, PtHome.Jt5, PtHome.Jt6});
    break;

  case Commands::Drive:
    Drive(PosFactory.DecodePosition(inputCmd.Content));
    break;

  case Commands::JDrive:
    Drive(PosFactory.DecodePosture(inputCmd.Content));
    break;

  case Commands::LDrive:
    LDrive(PosFactory.DecodePosition(inputCmd.Content));
    break;

  case Commands::Position:
    PosFactory.AddPosition(inputCmd.Content, PosState::Position);
    break;

  case Commands::Posture:
    PosFactory.AddPosition(inputCmd.Content, PosState::Posture);
    break;

  case Commands::ListPositions:
    PosFactory.LogPositions(PosState::Position);
    break;

  case Commands::ListPostures:
    PosFactory.LogPositions(PosState::Posture);
    break;

  case Commands::ListAllPositions:
    PosFactory.LogPositions(PosState::PosAndPost);
    break;
  case Commands::Help:
    Commands.List();
  }
}

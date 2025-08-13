#include "RobFrameWork.hpp"

RobFrame::RobFrame(){
  Kinematics.Init(this);
  Commands.Init(this);
  PosFactory.Init(this);

  Joint1.Init(this);
  Joint2.Init(this);
  Joint3.Init(this);
  Joint4.Init(this);
}

void RobFrame::Setup(){
  // Define Joint Structures
  Joint1.id = 1;
  Joint2.id = 2;
  Joint3.id = 3;
  Joint4.id = 4;
  // Attach Joint Parameters
  Joint1.attach(SERVO1_PIN,0,180,93);
  Joint2.attach(SERVO2_PIN,0,180,7);
  Joint3.attach(SERVO3_PIN,0,180,90);
  Joint4.attach(SERVO4_PIN,0,180,85);
  // Set Up Initial Configurations
  Joint1.write(PtHome.Jt1);
  Joint2.write(PtHome.Jt2);
  Joint3.write(PtHome.Jt3);
  Joint4.write(PtHome.Jt4);

  // Allocation of Static Position Data
  PosFactory.Setup();
}

/*Drive Joint to given Position*/
void RobFrame::GotoDeg(Joint& jt, int dest) {
  float origin = jt.read();
  Sweep(jt, origin, dest);
}

void RobFrame::Sweep(Joint& jt, int origin, int dest) {
  //log("Sweep Started");
  //log("Jt" + String(jt.id) + " at " + String(origin));
  //log("Jt" + String(jt.id) + " goto " + String(dest));

  if (dest >= origin){
    for (int i = origin; i <= dest; i += 1) {
      // in steps of 1 degree
      jt.write(i);
      Commands.Delay();
    }
  }else{
    for (int i = origin; i >= dest; i -= 1) {
      jt.write(i);
      Commands.Delay();
    }
  }
  //log("Sweep Done");
}

void RobFrame::Increment(Joint& jt, int origin, int dest, int itr){
  if (origin <= dest) {
    if (origin + itr <= dest) {
      jt.write(origin + itr);
      //log(origin + itr, "Fwd JT" + String(jt.id));
    }
  }
  else{
    if (origin  - itr >= dest){
      jt.write(origin - itr);
      //log(origin - itr, "Bwd JT" + String(jt.id));
    }
  }
}

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

/* Drive all Axis Simultanously to given Posture */
PosState RobFrame::Drive(Posture pt){
  log("Drive Data: " + String(pt.Jt1) +
      " | " + String(pt.Jt2) +
      " | " + String(pt.Jt3) +
      " | " + String(pt.Jt4));

  if(pt.Jt1 == 0 && pt.Jt2 == 0 && pt.Jt3 == 0 && pt.Jt4 == 0){
    return;
  }

  int origin1 = Joint1.read();
  int origin2 = Joint2.read();
  int origin3 = Joint3.read();
  int origin4 = Joint4.read();


  int delta1 = Joint1.delta(pt.Jt1);
  int delta2 = Joint2.delta(pt.Jt2);
  int delta3 = Joint3.delta(pt.Jt3);
  int delta4 = Joint4.delta(pt.Jt4);
  log("Drive Delta: " + String(delta1) +
      " | " + String(delta2) +
      " | " + String(delta3) +
      " | " + String(delta4));

  // Get max delta
  int delta = (abs(delta1) >= abs(delta2)) ? abs(delta1) : abs(delta2);
  delta = (abs(delta) >= abs(delta3)) ? abs(delta) : abs(delta3);
  delta = (abs(delta) >= abs(delta4)) ? abs(delta) : abs(delta4);

  // Iterate Through delta Range and Increment
  for(int itr = 1; itr <= delta; itr++){
    //log(itr, "iterator");
    Increment(Joint1, origin1, pt.Jt1, itr);
    Increment(Joint2, origin2, pt.Jt2, itr);
    Increment(Joint3, origin3, pt.Jt3, itr);
    Increment(Joint4, origin4, pt.Jt4, itr);
    // Delay with respect to Speed
    Commands.Delay();
  }
  return PosState::Defined;
}

/* Drive all Axis Simultanously to given position */
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

/* Drive all Axis Simultanously to given position */
PosState RobFrame::LDrive(Position ps){
  int valid = 1;
  int counter = 0;
  PosState result = PosState::Undefined;
  int range = Commands[Commands::Accuracy].Value;
  Position origin {};

  do{
    origin = Kinematics.fdKinematic(Posture {
        Joint1.read(),
        Joint2.read(),
        Joint3.read(),
        Joint4.read()
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

/*
 * Eingangs Daten aus Serialle Port lesen
 * Auswerten und entsprechend aus führen.
 */
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
        Joint1.read(),
        Joint2.read(),
        Joint3.read(),
        Joint4.read()
      });
    break;

  case Commands::Here:
    // Get Carthesian Koordinates
    log(Kinematics.fdKinematic(Posture {
          Joint1.read(),
          Joint2.read(),
          Joint3.read(),
          Joint4.read()
        }));
    break;

  case Commands::GoHome:
    Drive(Posture {PtHome.Jt1, PtHome.Jt2, PtHome.Jt3, PtHome.Jt4});
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

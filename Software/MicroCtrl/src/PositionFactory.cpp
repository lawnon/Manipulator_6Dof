/*
 * Program: Commands.cpp
 * Description: Class-Object containing prcedures
 *               for Parsing and validating Commands
 * Autor: Chukwunonso Bob-Anyeji
 * Date: 09.06.2024
 */

#include "RobFrameWork.hpp"
#include "PositionFactory.hpp"

PositionFactory::PositionFactory(){}
PositionFactory::~PositionFactory()
{
  log("PF: Destructor called.");
  delete _data;
}

/* Allocating and validation of Memory Block for Position Array */
void PositionFactory::Init(RobFrame* robFrame)
{
  _robFrame = robFrame;
  _data = new PosData[POS_LIMIT]{};
}

void PositionFactory::Setup(){
  if(_data != NULL && _robFrame != NULL){
    log("PF::Init: Initialization Done");
    LogInfo();
  }else{
    log(sizeof(_data)*POS_LIMIT, "PF::Init: Kritical Error Memory Allocation Failed:");
  }
}

void PositionFactory::LogInfo()
{
  log(sizeof(_data[0])*POS_LIMIT, "PF::LogInfo: Size of Memory-Block");
  log(sizeof(_data[0]),"PF::LogInfo: Size of Memory-Unit");
  //log(sizeof(_data[0].Identifier), "PF::LogInfo: Size of PosData.Identifier");
  //log(sizeof(_data[0].Position), "PF::LogInfo: Size of PosData.Position");
  //log(sizeof(_data[0].Posture), "PF::LogInfo: Size of PosData.Posture");
  //log((sizeof(_data[0])*POS_LIMIT)/sizeof(_data[0]), "PF::LogInfo: Nr of Memory-Units => ");
  //log(sizeof(char), "PF::LogInfo: Size of Default Char");
  //log(_definedCount, "PF::LogInfo: Number of defined Positions");
}

void PositionFactory::LogPositions(PosState state){
  log(_definedCount, "PF::LogPositions: Number of defined Positions");
  for(int itr = 0; itr < _definedCount; itr++){
    log(_data[itr], state);
  }
}

PosState PositionFactory::Decode(String content, char delimiter = ','){
  PosState decodeState = PosState::Undefined;

  for(int itr = 0;itr < content.length();itr++){
    if(isAlpha(content.charAt(itr))){
      //log("PF::Decode");
      if(content.equals("here")){
        //log("PF::Decode: here");
      }else if(content.equals("joints")){
        //log("PF::Decode: joints");
      }
      return decodeState;
    }
  }
}

/* Transforem String Array in to Posture Structure */
Posture PositionFactory::DecodePosture(String input, String filter = ","){
  /* Expected String := "<jt1>, <jt2, <jt3>, <jt4>" */

  if(input.length() <= 0 ){
    return {0, 0, 0, 0};
  }

  if(input.equals(_robFrame->Commands[Commands::Here].Name)){
    return {
      _robFrame->Joint1.read(),
      _robFrame->Joint2.read(),
      _robFrame->Joint3.read(),
      _robFrame->Joint4.read()
    };
  }

  int8_t delimeter1 = input.indexOf(filter);
  int8_t delimeter2 = input.indexOf(filter, delimeter1 + 1);
  int8_t delimeter3 = input.indexOf(filter, delimeter2 + 1);

  Posture pt{
    input.substring(0, delimeter1).toFloat(),
    input.substring(delimeter1 + 1, delimeter2).toFloat(),
    input.substring(delimeter2 + 1, delimeter3).toFloat(),
    input.substring(delimeter3 + 1, input.length()).toFloat()
  };

  return pt;
}

/* Transform String Arry in to Position Sturcture  */
Position PositionFactory::DecodePosition(String input, String filter = ","){
  /* Expected String := "<x>, <y, <z, <a>, <b>, <c>" */

  if(input.length() <= 0 ){
    return {0, 0, 0, 0, 0, 0};
  }

  if(input.equals(_robFrame->Commands[Commands::Here].Name)){
    return {_robFrame->Kinematics.fdKinematic(Posture{
          _robFrame->Joint1.read(),
          _robFrame->Joint2.read(),
          _robFrame->Joint3.read(),
          _robFrame->Joint4.read()
        })};
  }

  int8_t delimeter1 = input.indexOf(filter);
  int8_t delimeter2 = input.indexOf(filter, delimeter1 + 1);
  int8_t delimeter3 = input.indexOf(filter, delimeter2 + 1);
  int8_t delimeter4 = input.indexOf(filter, delimeter3 + 1);
  int8_t delimeter5 = input.indexOf(filter, delimeter4 + 1);

  Position pos{
    input.substring(0, delimeter1).toFloat(),
    input.substring(delimeter1 + 1, delimeter2).toFloat(),
    input.substring(delimeter2 + 1, delimeter3).toFloat(),
    input.substring(delimeter3 + 1, delimeter4).toFloat(),
    input.substring(delimeter4 + 1, delimeter5).toFloat(),
    input.substring(delimeter5 + 1, input.length()).toFloat()
  };

  return pos;
}

void PositionFactory::Update(int index, String& params, PosState state){
  // Update Positions
  switch(state){
  case PosState::Position:
    _data[index].Position = TosPosition(DecodePosition(params));
    break;
  case PosState::Posture:
    _data[index].Posture = TosPosture(DecodePosture(params));
    break;
  }
}

int& PositionFactory::AddPosition(String content, PosState state,  char filter = ' '){
  /* Expected String := "Position <identifier> <x>, <y>, <z>, <a>, <b>, <c>" */

  int result = -1;
  int delimeter = content.indexOf(filter);
  String identifier = content.substring(0, fmax(content.length(), IDENTIFIER_LEN));
  String params = "";

  if(delimeter >= 1){
    identifier = content.substring(0, fmin(delimeter, IDENTIFIER_LEN));
    params = content.substring(delimeter + 1, content.length());
  }

  //log("PF::AddPosition: Content: " + content);
  //log("PF::AddPosition: Identifier: " + identifier);
  //log("PF::AddPosition: Parameters: " + params);
  //log(delimeter, "PF::AddPosition: Delimeter ");

  if(identifier.length() <= 0 ||
     identifier.equals(_robFrame->Commands.Get(Commands::Here).Name) ||
     identifier.equals(_robFrame->Commands.Get(Commands::Joints).Name)){
    log("PF::AddPosition: Invalid Identifier");
    return result;
  }

  char cIdentifier[IDENTIFIER_LEN];
  identifier.toCharArray(cIdentifier, IDENTIFIER_LEN);

  for(int itr = 0; itr < _definedCount; itr++){
    if(strcmp(_data[itr].Identifier, cIdentifier) == 0){
      result = itr;
      Update(itr, params, state);
      log(_data[itr], state);
      return result;
    }
  }

  if(RangeCheck() <= 0){
    log("PF::AddPos: Range Exceeded");
    return result;
  }

  strcpy(_data[_definedCount].Identifier, cIdentifier);
  Update(_definedCount, params, state);

  log(_data[_definedCount], state);
  _definedCount++;

  result = _definedCount - 1;
  return result;
}

int& PositionFactory::Count(){
  return _definedCount;
}

void PositionFactory::Get(String identifier){
  log("PF::Get not Implemented");
}
void PositionFactory::Del(String identifier){
  log("PF::Del not Implemented");
}

int8_t PositionFactory::RangeCheck(){
  if(_definedCount >= POS_LIMIT){
    log("PF::RangCheck: Critical Error");
    return -1;
  }
  return 1;
}

sPosition PositionFactory::TosPosition(Position ps){
  return{ps.X, ps.Y, ps.Z, ps.A, ps.B, ps.C};
}
sPosture PositionFactory::TosPosture(Posture pt){
  return{pt.Jt1, pt.Jt2, pt.Jt3, pt.Jt4};
}

Position PositionFactory::operator[](int index){
  return {
    (float)_data[index].Position.X,
    (float)_data[index].Position.Y,
    (float)_data[index].Position.Z,
    (float)_data[index].Position.A,
    (float)_data[index].Position.B,
    (float)_data[index].Position.C
  };
}

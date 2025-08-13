/*
 * Program: Types.hpp
 * Description: header file containing Global Structs
 * Autor: Chukwunonso Bob-Anyeji
 * Date: 09.06.2024
 */

#ifndef TYPES_H_
#define TYPES_H_

#define IDENTIFIER_LEN 16

#include <WString.h>

/* Kinematik Struct */

/* Denavit Hartenberg-Parameter for all Joints */
struct DhParameters{
  float alpha[4] = {90.000, 180.000, -180.000,  0.000};
  float link[4]  = {28.691,  58.000,   68.300, 66.539};
  float disp[4]  = { 0.000,   0.000,    0.000,  0.000};
  float theta[4] = { 0.000,   0.000,    0.000,  0.000};
};

/* Denavit Hartenberg_Parameter Discription */
struct DhParam{
  float alpha;
  float link;
  float disp;
  float theta;
};

/* Structs Containing a 4 by 4 Matrix */
struct Matrix4x4{
  float m11[4]; float m12[4]; float m13[4]; float m14[4];
};

/* Position Data State */
enum class PosState{
  Invalid = -1,
  Undefined = 0,
  Defined = 1,
  Position = 4,
  Posture = 8,
  PosAndPost = 16
};

/* Location and Orientation Struct */
struct Position{
  float X;
  float Y;
  float Z;
  float A;
  float B;
  float C;
};
/* Location and Orientation Struct */
struct sPosition{
  short X;
  short Y;
  short Z;
  short A;
  short B;
  short C;
};

/* Angular Constelation of Joints */
struct Posture{
  float Jt1;
  float Jt2;
  float Jt3;
  float Jt4;
};
/* Angular Constelation of Joints */
struct sPosture{
  short Jt1;
  short Jt2;
  short Jt3;
  short Jt4;
};

/* Position and Posture Data */
struct PosData{
  char Identifier[IDENTIFIER_LEN];
  sPosition Position;
  sPosture Posture;
};

/* Command Struct */
struct Command {
  int Id;
  String Name;
  int Value;
  String Content;
};

#endif // TYPES_H_

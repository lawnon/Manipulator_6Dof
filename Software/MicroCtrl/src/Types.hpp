/*
 * Program: Types.hpp
 * Description: header file containing Global Structs
 * Autor: Chukwunonso Bob-Anyeji
 * Date: 09.06.2024
 */

// NOTE:
// 1) Grosse eines Pointers im ATM-2560 Kontrollern sind 16bit
// 2) Das System/Compiler basiert auf dem C++ 11 mit einem LP32 Daten Modell d.h.
// 2.1) Charakter => 8 Bit
// 2.2) Integer => 16 Bit
// 2.3) Long Interger => 32 Bit
// 2.4) Long Long Interger => 64 Bit

#ifndef TYPES_H_
#define TYPES_H_

// Type Definitionen
using int8 = signed char ;
using uint8 = unsigned char;
using byte = unsigned char;

using int16 = short int;
using uint16 = unsigned short int;
using word = unsigned int;

using int32 = long int;
using uint32 = unsigned long int;

using int64 = long long int;
using uint64 = unsigned long long int;

// Variable Definitionen
#define IDENTIFIER_LEN 16
#define OK 1
#define NOK -1
#define ON 1
#define OFF 0

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

/*
 * Program: Kinematic.hpp
 * Description: Header file containing prcedures
 *               for Forward and Backward Kinematics
 * Autor: Chukwunonso Bob-Anyeji
 * Date: 09.06.2024
 */

#ifndef KINEMATIC_H_
#define KINEMATIC_H_

#include <math.h>
#include <HardwareSerial.h>
#include <WString.h>

#include "Types.hpp"
#include "Logger.hpp"


// Misc
float getRad(float deg);
float getDeg(float rad);

class RobFrame;
class Kinematic
{
private:
  RobFrame* _robFrame;

  Matrix4x4 jointTMatrix(DhParam parma);
  Matrix4x4 matrixMulp(Matrix4x4 A, Matrix4x4 B);
  Matrix4x4 armTMatrix(Posture posture);
public:
  Kinematic();

  void Init(RobFrame* robFrame);
  void printMatrix(Matrix4x4 mat, String heading);
  Position fdKinematic(Posture posture);
  Posture ivKinematic(Position position);

  // Danavit Hartenberg Parameter
  DhParam dhPar1 = {};
  DhParam dhPar2 = {};
  DhParam dhPar3 = {};
  DhParam dhPar4 = {};

  // Achsen-Transformations Matrizen
  Matrix4x4 tMat01 = {};
  Matrix4x4 tMat12 = {};
  Matrix4x4 tMat23 = {};
  Matrix4x4 tMat34 = {};
  // Verkettet Transformations Matrizen
  Matrix4x4 tMat02 = {};
  Matrix4x4 tMat03 = {};
  Matrix4x4 tMat04 = {};
};

#endif // KINEMATIC_H_

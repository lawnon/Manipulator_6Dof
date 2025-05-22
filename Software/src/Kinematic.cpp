/*
 * Program: Kinematic.cpp
 * Description: Class-Object containing prcedures
 *               for Forward and Backward Kinematics
 * Autor: Chukwunonso Bob-Anyeji
 * Date: 09.06.2024
 */

#include "RobFrameWork.hpp"
#include "Kinematic.hpp"

const float pi = 3.1416;

// Degree to Radiant Conversion
float getRad(float deg){
  return deg * (pi/180);
}

// Radiant to Degree Conversion
float getDeg(float rad){
  return rad * (180/pi);
}

// Assignment of D-H Parameters
Kinematic::Kinematic(){
  Kinematic::dhPar1 = { 90.000, 28.691, 0.000, 0.000};
  Kinematic::dhPar2 = {-180.000, 58.000, 0.000, 0.000};
  Kinematic::dhPar3 = { 180.000, 68.300, 0.000, 0.000};
  Kinematic::dhPar4 = {   0.000, 66.539, 0.000, 0.000};
}

void Kinematic::Init(RobFrame* robFrame){
  _robFrame = robFrame;
}

// Print 4x4 Matrix to Serial Monitor
void Kinematic::printMatrix(Matrix4x4 mat, String heading){
  //log("=============================");
  log(heading);

  for(int i = 0; i <= 3; i++){
    log("Mat" + String(i+1) + "1: " + String(mat.m11[i]) +
        "| Mat" + String(i+1) + "2: " + String(mat.m12[i]) +
        "| Mat" + String(i+1) + "3: " + String(mat.m13[i]) +
        "| Mat" + String(i+1) + "4: " + String(mat.m14[i]));
  }
}

// Forward-Kinematics calculation with respect to given Posture
Position Kinematic::fdKinematic(Posture pt){
  Matrix4x4 mat = Kinematic::armTMatrix(pt);
  //printMatrix(mat, "Arm Transformation Matrix");

  Position ps {};
  ps.X = mat.m14[0];
  ps.Y = mat.m14[1];
  ps.Z = mat.m14[2];

  if(mat.m11[0] == 0 && mat.m11[1] == 0){
    ps.A = getDeg(atan2(mat.m12[0], mat.m12[1]));
    ps.B = 0;
    ps.C = 0;
  }
  else {
    ps.A = getDeg(atan2(mat.m12[2], mat.m13[2]));
    ps.B = getDeg(atan2(-mat.m11[2], sqrtf(powf(mat.m11[0],2) + powf(mat.m11[1],2))));
    ps.C = getDeg(atan2(mat.m11[1], mat.m11[0]));
  }

  return ps;
}

// Inverse-Kinematics calculation with respect to given Position
Posture Kinematic::ivKinematic(Position p4){
  /*
   * Determination of joint rotations through
   * Trigonometrical-Ratios (SOH-CAH-TOA),
   * Pythagorean-Theorem (hyp^2 = adj^2 + opp^2) and
   * Linear combination of Sin and Cosin waves ie.
   * => A*cos(x) + B*sin(x) = C*cos(x + phi) with
   * => C = sgn(A)*Sqrt(A^2 + B^2) ie. sgn(A) = (A/|A|)
   * => phi = arctan(-B/A)
  */

  Posture pt = {};
  logln(p4, "ivKinematic: Target Position");

  // Determination of joint 1 rotation
  float jt1 = atan2(p4.Y, p4.X);
  jt1 = (jt1 >= pi/2 || jt1 <= -pi/2) ? (jt1 - pi) : jt1;

  pt.Jt1 = getDeg(jt1);
  // logft(jt1, "jt1");
  // logft(pt.jt1, "pt.jt1");

  // Calculation of joint 3 position
  float adj34 = dhPar4.link*cos(getRad(p4.B));
  float opp34 = dhPar4.link*sin(getRad(p4.B));
  Position p3 = {
    p4.X - adj34*cos(jt1),
    p4.Y - adj34*sin(jt1),
    p4.Z + opp34,
  };
  // logft(dhPar4.link, "link 4");
  // logft(adj34, "adj34");
  // logft(opp34, "opp34");
  // log(p3, "Location of Joint 3");

  // Determination of joint 2 rotation
  // Total length of Adjacents and opposits up to joint 3
  float adj3 = p3.X/cos(jt1);
  float opp3 = p3.Z;
  // Linear combinationf constants
  float A = adj3 - dhPar1.link;
  float B = opp3;
  // logft(adj3, "adj3");
  // logft(opp3, "opp3");
  // logft(A, "A");
  // logft(B, "B");
  // Application of linear combination
  float nominator = fabs(A)*(powf(dhPar2.link,2) - powf(dhPar3.link,2) + powf(A,2) + powf(opp3,2));
  float denominator = 2*dhPar2.link*A*sqrtf(powf(A,2)+powf(B,2));
  float jt2 = acos(nominator/denominator) - atan(-B/A);
  pt.Jt2 = getDeg(jt2);
  // logft(jt2, "jt2");
  // logft(pt.jt2, "pt.jt2");

  // Determination of joint 3 rotation
  // Calculation of joint 2 position
  float adj23 = adj3 - (dhPar2.link*cos(jt2) + dhPar1.link);
  float opp23 = opp3 - (dhPar2.link*sin(jt2));
  Position p2 = {
    p3.X - adj23*cos(jt1),
    p3.Y - adj23*sin(jt1),
    p3.Z - opp23,
  };
  // logft(adj23, "adj23");
  // logft(opp23, "opp23");
  // log(p2, "Location of Joint 2");
  // Calculation of joint 3 rotation with respect to joint 2 and 3 positions
  float jt3 = (p3.Z >= p2.Z) ? (jt2 - acos(adj23/dhPar3.link)) : (jt2 + acos(adj23/dhPar3.link));
  pt.Jt3 = getDeg(jt3);
  // logft(jt3, "jt3");
  // logft(pt.jt3, "pt.jt3");

  // Determination of joint 4 rotation
  // Calculation of joint 3 rotation with respect to joint 3 and 4 positions
  float jt4 = (p4.Z >= p3.Z) ?
    (acos(adj34/dhPar4.link)+jt3-jt2) : (-acos(adj34/dhPar4.link)+jt3-jt2);
  pt.Jt4 = getDeg(jt4);
  // logft(jt4, "jt4");
  // logft(pt.jt4, "pt.jt4");

  return pt;
}

// Returns the product of given Matrices matA and matB
Matrix4x4 Kinematic::matrixMulp(Matrix4x4 matA, Matrix4x4 matB){
  Matrix4x4 matAB;

  float Mat[4][4] = {};

  // Matrix m11 bis m41
  matAB.m11[0] = matA.m11[0]*matB.m11[0] + matA.m12[0]*matB.m11[1] + matA.m13[0]*matB.m11[2] + matA.m14[0]*matB.m11[3];
  matAB.m11[1] = matA.m11[1]*matB.m11[0] + matA.m12[1]*matB.m11[1] + matA.m13[1]*matB.m11[2] + matA.m14[1]*matB.m11[3];
  matAB.m11[2] = matA.m11[2]*matB.m11[0] + matA.m12[2]*matB.m11[1] + matA.m13[2]*matB.m11[2] + matA.m14[2]*matB.m11[3];
  matAB.m11[3] = matA.m11[3]*matB.m11[0] + matA.m12[3]*matB.m11[1] + matA.m13[3]*matB.m11[2] + matA.m14[3]*matB.m11[3];
  // Matrix m12 bis m42
  matAB.m12[0] = matA.m11[0]*matB.m12[0] + matA.m12[0]*matB.m12[1] + matA.m13[0]*matB.m12[2] + matA.m14[0]*matB.m12[3];
  matAB.m12[1] = matA.m11[1]*matB.m12[0] + matA.m12[1]*matB.m12[1] + matA.m13[1]*matB.m12[2] + matA.m14[1]*matB.m12[3];
  matAB.m12[2] = matA.m11[2]*matB.m12[0] + matA.m12[2]*matB.m12[1] + matA.m13[2]*matB.m12[2] + matA.m14[2]*matB.m12[3];
  matAB.m12[3] = matA.m11[3]*matB.m12[0] + matA.m12[3]*matB.m12[1] + matA.m13[3]*matB.m12[2] + matA.m14[3]*matB.m12[3];
  // Matrix m13 bis m43
  matAB.m13[0] = matA.m11[0]*matB.m13[0] + matA.m12[0]*matB.m13[1] + matA.m13[0]*matB.m13[2] + matA.m14[0]*matB.m13[3];
  matAB.m13[1] = matA.m11[1]*matB.m13[0] + matA.m12[1]*matB.m13[1] + matA.m13[1]*matB.m13[2] + matA.m14[1]*matB.m13[3];
  matAB.m13[2] = matA.m11[2]*matB.m13[0] + matA.m12[2]*matB.m13[1] + matA.m13[2]*matB.m13[2] + matA.m14[2]*matB.m13[3];
  matAB.m13[3] = matA.m11[3]*matB.m13[0] + matA.m12[3]*matB.m13[1] + matA.m13[3]*matB.m13[2] + matA.m14[3]*matB.m13[3];
  // Matrix m12 bis m42
  matAB.m14[0] = matA.m11[0]*matB.m14[0] + matA.m12[0]*matB.m14[1] + matA.m13[0]*matB.m14[2] + matA.m14[0]*matB.m14[3];
  matAB.m14[1] = matA.m11[1]*matB.m14[0] + matA.m12[1]*matB.m14[1] + matA.m13[1]*matB.m14[2] + matA.m14[1]*matB.m14[3];
  matAB.m14[2] = matA.m11[2]*matB.m14[0] + matA.m12[2]*matB.m14[1] + matA.m13[2]*matB.m14[2] + matA.m14[2]*matB.m14[3];
  matAB.m14[3] = matA.m11[3]*matB.m14[0] + matA.m12[3]*matB.m14[1] + matA.m13[3]*matB.m14[2] + matA.m14[3]*matB.m14[3];

  return matAB;
}

/* Returns Joint-Transformation Matrix with respect
   to given of Dh-Parametersd*/
Matrix4x4 Kinematic::jointTMatrix(DhParam dhPar){
  Matrix4x4 mat;
  /*
   * Denavit-Hartenberg-Transformationmatrix (a alpha d theta)
   * T = [
   *      cos(the), -sin(the)*round(cos(alp)), sin(the)*sin(alp),  a*cos(the);
   *      sin(the), cos(the)*round(cos(alp)),  -cos(the)*sin(alp), a*sin(the);
   *      0,        sin(alp),                  round(cos(alp)),    d;
   *      0,        0,                         0,                  1
   *     ];
   */
  // Matrix m11 bis m41
  mat.m11[0] = cos(getRad(dhPar.theta));
  mat.m11[1] = sin(getRad(dhPar.theta));
  mat.m11[2] = 0;
  mat.m11[3] = 0;
  // Matrix m12 bis m42
  mat.m12[0] = -cos(getRad(dhPar.alpha))*sin(getRad(dhPar.theta));
  mat.m12[1] = cos(getRad(dhPar.alpha))*cos(getRad(dhPar.theta));
  mat.m12[2] = sin(getRad(dhPar.alpha));
  mat.m12[3] = 0;
  // Matrix m13 bis m43
  mat.m13[0] = sin(getRad(dhPar.alpha))*sin(getRad(dhPar.theta));
  mat.m13[1] = -sin(getRad(dhPar.alpha))*cos(getRad(dhPar.theta));
  mat.m13[2] = cos(getRad(dhPar.alpha));
  mat.m13[3] = 0;
  // Matrix m14 bis m44
  mat.m14[0] = dhPar.link*cos(getRad(dhPar.theta));
  mat.m14[1] = dhPar.link*sin(getRad(dhPar.theta));
  mat.m14[2] = dhPar.disp;
  mat.m14[3] = 1;

  return mat;
}

/*Returns Linked Transformation-Matrix of Robotic-Arm
  with respect to given Posture */
Matrix4x4 Kinematic::armTMatrix(Posture p){
  // D-H Parameter Zuweisen
  dhPar1.theta = p.Jt1;
  dhPar2.theta = p.Jt2;
  dhPar3.theta = p.Jt3;
  dhPar4.theta = p.Jt4;

  //log("===================================================");
  //log("D-H Paramter");
  //log("dhPar1: " + String(dhPar1.alpha) + " ," + String(dhPar1.link) + ", " + String(dhPar1.disp) + ", " + String(dhPar1.theta));
  //log("dhPar2: " + String(dhPar2.alpha) + " ," + String(dhPar2.link) + ", " + String(dhPar2.disp) + ", " + String(dhPar2.theta));
  //log("dhPar3: " + String(dhPar3.alpha) + " ," + String(dhPar3.link) + ", " + String(dhPar3.disp) + ", " + String(dhPar3.theta));
  //log("dhPar4: " + String(dhPar4.alpha) + " ," + String(dhPar4.link) + ", " + String(dhPar4.disp) + ", " + String(dhPar4.theta));
  //log("===================================================");

  // Achsen-Transformations Matrizen Ermittlen
  tMat01 = jointTMatrix(dhPar1);
  tMat12 = jointTMatrix(dhPar2);
  tMat23 = jointTMatrix(dhPar3);
  tMat34 = jointTMatrix(dhPar4);

  // Achsen Transformationen Loggen
  //printMatrix(tMat01, "TMatrix01");
  //printMatrix(tMat12, "TMatrix12");
  //printMatrix(tMat23, "TMatrix23");
  //printMatrix(tMat34, "TMatrix34");

  // Verkettet-Transformations Matrizen Ermittlen
  tMat02 = matrixMulp(tMat01, tMat12);
  tMat03 = matrixMulp(tMat02, tMat23);
  tMat04 = matrixMulp(tMat03, tMat34);

  // Transformationen Loggen
  //printMatrix(tMat02, "TMatrix02");
  //printMatrix(tMat03, "TMatrix03");
  //printMatrix(tMat04, "TMatrix04");

  return tMat04;
}

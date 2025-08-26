/*
 * Program: Kinematic.cpp
 * Description: Class-Object containing prcedures
 *               for Forward and Backward Kinematics
 * Autor: Chukwunonso Bob-Anyeji
 * Date: 09.06.2024
 */

#include "RobFrameWork.hpp"
#include "Kinematic.hpp"

using namespace Logger;

const float pi = 3.1416;

// Degree to Radiant Conversion
float getRad(float deg){
  return deg * (pi/180);
}

// Radiant to Degree Conversion
float getDeg(float rad){
  return rad * (180/pi);
}

// Sign Function
float sign(float val){
  return val/abs(val);
}

// Assignment of D-H Parameters
Kinematic::Kinematic(){
  Kinematic::dhPar1 = {90.000, -90.000, 0.000,   0.000, 0.000};
  Kinematic::dhPar2 = { 0.000,   0.000, 0.000, 258.300, 0.000};
  Kinematic::dhPar3 = { 0.000,  90.000, 0.000, 281.516, 0.000};
  Kinematic::dhPar4 = { 0.000, -90.000, 0.000,   0.000, 0.000};
  Kinematic::dhPar5 = { 0.000,  90.000, 0.000,  74.710, 0.000};
  Kinematic::dhPar6 = { 0.000,   0.000, 0.000,   0.000, 0.000};
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
  //Matrix4x4 mat = armTMatrix(pt);
  //printMatrix(mat, "Arm Transformation Matrix");

  Position ps {};
  //ps.X = mat.m14[0];
  //ps.Y = mat.m14[1];
  //ps.Z = mat.m14[2];

  //ps.B = getDeg(atan2(-mat.m11[2], sqrtf(powf(mat.m11[0],2) + powf(mat.m11[1],2))));
  //ps.A = getDeg(atan2(mat.m11[1]/cos(getRad(ps.B)), mat.m11[0]/cos(getRad(ps.B))));
  //ps.C = getDeg(atan2(mat.m12[2]/cos(getRad(ps.B)), mat.m13[3]/cos(getRad(ps.B))));

  if(ps.B == (getDeg(pi)/2)){
    ps.A = 0;
    //ps.C = getDeg(atan2(mat.m12[0], mat.m12[1]));
  }
  if((ps.B == -(getDeg(pi)/2))) {
    ps.A = 0;
    //ps.C = getDeg(-atan2(mat.m12[0], mat.m12[1]));
  }

  return ps;
}

// Ermittlung von theta2 durch Anwendung der Linear Kombination von Sinus
// und Kosinus wellen
// ie. A*cos(x) + B*sin(x) = C*cos(x + phi) with
// C = sgn(A).Sqrt(A^2 + B^2) ie. sgn(A) = (A/|A|)
// phi = arctan(-B/A)
Posture Kinematic::ivPosition(Position ps){
  Posture pt = {};
  logln(ps, "ivPosition: Target Position");

  // Berechnung von theata 1
  float jt1 = atan2(ps.Y, ps.X);
  pt.Jt1 = getDeg(jt1);

  logft(jt1, "jt1");
  logft(pt.Jt1, "pt.jt1");

  // Berechnung von theta 2
  float A = 2*ps.Z*dhPar2.link;
  float B = -2*ps.X*cos(jt1)*dhPar2.link-2*ps.Y*sin(jt1)*dhPar2.link+2*dhPar1.link*dhPar2.link;
  float C = sign(A)*sqrt(powf(A,2)+powf(B,2));
  float D = -powf(dhPar3.link,2)+(powf(ps.X,2)*powf(cos(jt1),2))+(powf(ps.Y,2)*powf(sin(jt1),2))
    +(2*ps.X*cos(jt1)*ps.Y*sin(jt1))-(2*ps.X*cos(jt1)*dhPar1.link)-(2*ps.Y*sin(jt1)*dhPar1.link)
    +powf(dhPar2.link,2)+powf(dhPar1.link,2)+powf(ps.Z,2);
  float Phi = atan2(-B,A);
  // d.h
  float jt2 = atan2(sqrt(1-powf(D/C,2)),(D/C)) - Phi;
  float jt2_neg = atan2(-sqrt(1-powf(D/C,2)),(D/C)) - Phi;

  pt.Jt2 = getDeg(jt2);

  logft(jt2, "jt2");
  logft(pt.Jt2, "pt.jt2");

  // Berechnung von theta 3:
  float jt3 = atan2(-ps.X*cos(jt1)-ps.Y*sin(jt1)-dhPar2.link*sin(jt2)+dhPar1.link, (ps.Z-dhPar2.link*cos(jt2)))-jt2;
  pt.Jt3 = getDeg(jt3);

  logft(jt3, "jt3");
  logft(pt.Jt3, "pt.jt3");
}

// Ermittlung der Orientierung/Gelenkstellung d.h. th4, th5 und th6
//
// mit TMati_j = |RotationsMatrix(3x3) Verschiebung(3x1)|
//               |      0(1x3)               1(1x1)     |
//
// Berechnung von theta 4
Posture Kinematic::ivOrientation(Posture pt, Position ps){
  DhParam rmatPar = {ps.A, ps.B, ps.C, 0.000, 0.000};
  //Matrix4x4 rmat = jointTMatrix(rmatPar);

  // Berechnung von theta 4
  //pt.Jt4 = getDeg(atan2(-rmat.m13[0],rmat.m13[1]));

  // Berechnung von theta 5
  //pt.Jt5 = getDeg(atan2(sqrt(powf(rmat.m13[0],2)+powf(rmat.m13[1],2)),rmat.m13[2]));

  // Berechnung von theta 6
  //pt.Jt6 = getDeg(atan2(rmat.m11[2],rmat.m12[2]));

  return pt;
}

// Inverse-Kinematics calculation with respect to given Position
Posture Kinematic::ivKinematic(Position p6){
  Posture pt = {};

  // Berechnung der Positionsgebenden Gelenkstellung d.h. th1, th2 und th3
  //
  // TMat = |RotationsMatrix(3x3) Verschiebung(3,1)|
  //        |      0                     1         |
  //
  // TMat03 = TMat1*TMat2*TMat3
  // TMat03 =...
  //     |-sin(th1) -cos(th1)*sin(th2+th3) cos(th1)*cos(th2+th3) cos(th1)*(a(3)*cos(th2+th3)+a(2)*cos(th2)+a(1))|
  //     | cos(th1) -sin(th1)*sin(th2+th3) sin(th1)*cos(th2+th3) sin(th1)*(a(3)*cos(th2+th3)+a(2)*cos(th2)+a(1))|
  //     |    0          cos(th2+th3)           cos(th2+th3)            a(3)*sin(th2+th3)+a(2)*sin(th2)         |
  //     |    0              0                      0                                  1                        |
  //
  pt = ivPosition(p6);


  // Berechnung der Orientierungsgebenden Gelenkstellung d.h. th4, th5 und th6
  // RMat0_6 = RMat0_3 * RMat3_6
  // (RMat0_3)^-1 * RMat0_6 = (RMat0_3)^-1 * RMat0_3 * RMat3_6
  // d.h.
  // RMat3_6 = (RMat0_3)^-1 * RMat0_6
  //
  // mit TMat0_6 = |RotationsMatrix(3x3) Verschiebung(3x1)|
  //               |      0(1x3)                1         |
  //
  // TMat36 = TMat4*TMat5*TMat6
  // TMat36 =...
  //     |-cos(th4)*cos(th6)-sin(th4)*cos(th5)*sin(th6) cos(th4)*sin(th6)-sin(th4)*cos(th5)*cos(th6) -sin(th4)*sin(th5) -a(6)*cos(th4)*cos(th6)-a(6)*sin(th4)*cos(th5)*sin(6)-a(5)*sin(th4)*sin(th5)+a(4)*cos(4)|
  //     |-sin(th4)*cos(th6)+cos(th4)*cos(th5)*sin(th6) sin(th4)*sin(th6)+cos(th4)*cos(th5)*cos(th6)  cos(th4)*sin(th5) -a(6)*sin(th4)*cos(th6)-a(6)*cos(th4)*cos(th5)*sin(6)-a(5)*cos(th4)*sin(th5)+a(4)*sin(4)|
  //     |            sin(th5)*sin(th6)                             sin(th5)*cos(th6)                    -cos(th5)                                a(6)*sin(th5)*sin(th6)-a(5)*cos(th5)                          |
  //     |                    0                                             0                                0                                                    0                                             |
  //
  // Ermittlung der Manipulator Transformations-Matrix entsprechend der
  // gegebenen Positions-Winkelwerten
  pt = ivOrientation(pt,p6);

  // Ermittlung der Positions Difference zwischen Endeffektor und Gelenk 5
  // unterberucksichtigung der Nickung in Acshe 2, 3 oder 5
  if(pt.Jt2 || pt.Jt3 || pt.Jt5){
    // Positions Difference berechnen
    Position psFwd1 = fdKinematic(pt);

    // Gelenk 5 entlang der Orientierungs Achse verschieben
    // Matrix4x4  = tmat06v*TransFK(0,90,0,0,0)*TransFK(0,0,0,2*a(5),0);
    //DhParam tOpp = {0.000, 90.000, 0.000, 0.000, 0.000};
    //Matrix
    //DhParam tDis = {0.000,  0.000, 0.000, 2*dhPar5.link, 0.000};
    //Matrix4x4 arm = armTMatrix(pt);
    //Matrix4x4 armOpp = matrixMulp(arm, tOpp);
    //Matrix4x4 armOppDis = matrixMulp(armOpp, tDis);

    // Berechnung der Positionsgebenden Gelenkstellung d.h. th1, th2 und th3
    // unter Berücksichtigung von Achse 56
    //Position psFwd2 = {armOppDis.m14[0], armOppDis.m14[1], armOppDis.m14[2],0,0,0};
    //Posture ptFwd = Position(psFwd2);

    //// Berechnung der Orientierung
    //ptFwd = Orientation(ptFwd, );
    //th4 = ToDeg(thr4);
    //th5 = ToDeg(thr5);
    //th6 = ToDeg(thr6);
  }


  logln(p6, "ivKinematic: Target Position");
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


// Returns Joint-Transformation Matrix with respect
// to given of Dh-Parameters
tmm::Matrix<4,4> Kinematic::jointTMatrix(DhParam dhPar){
  // Denavit-Hartenberg-Transformationmatrix (a alpha beta d theta)
  //
  //TMat = [ cos(beta)*cos(gamma) -cos(alpha)*sin(gamma)+sin(alpha)*sin(beta)*cos(gamma)  sin(alpha)*sin(gamma)+cos(alpha)*sin(beta)*cos(gamma) a*cos(gamma);
  //         cos(beta)*sin(gamma)  cos(alpha)*cos(gamma)+sin(alpha)*sin(beta)*sin(gamma) -sin(alpha)*cos(gamma)+cos(alpha)*sin(beta)*sin(gamma) a*sin(gamma);
  //        -sin(beta)             sin(alpha)*cos(beta)                                   cos(alpha)*cos(beta)                                  d           ;
  //         0                     0                                                      0                                                     1           ];
  //
  //TMat = [m[0][0], m[0][1], m[0][2], m[0][3];
  //        m[1][0], m[1][1], m[1][2], m[1][3];
  //        m[2][0], m[2][1], m[2][2], m[2][3];
  //        m[3][0], m[3][1], m[3][2], m[3][3];]
  //

  tmm::Matrix<4,4> mat = {};

  // Matrix m11 bis m41
  mat.printTo(Serial);
  mat[0][0] = cos(getRad(dhPar.beta))*cos(getRad(dhPar.theta));
  mat[1][0] = cos(getRad(dhPar.beta))*sin(getRad(dhPar.theta));
  mat[2][0] = -sin(getRad(dhPar.beta));
  mat[3][0] = 0;
  // Matrix m12 bis m42
  mat[0][1] = -cos(getRad(dhPar.alpha))*sin(getRad(dhPar.theta))+sin(getRad(dhPar.alpha))*sin(getRad(dhPar.beta))*cos(getRad(dhPar.theta));
  mat[1][1] =  cos(getRad(dhPar.alpha))*cos(getRad(dhPar.theta))+sin(getRad(dhPar.alpha))*sin(getRad(dhPar.beta))*sin(getRad(dhPar.theta));
  mat[2][1] =  sin(getRad(dhPar.alpha))*cos(getRad(dhPar.beta));
  mat[3][1] =  0;
  // Matrix m13 bis m43
  mat[0][2] =  sin(getRad(dhPar.alpha))*sin(getRad(dhPar.theta))+cos(getRad(dhPar.alpha))*sin(getRad(dhPar.beta))*cos(getRad(dhPar.theta));
  mat[1][2] = -sin(getRad(dhPar.alpha))*cos(getRad(dhPar.theta))+cos(getRad(dhPar.alpha))*sin(getRad(dhPar.beta))*sin(getRad(dhPar.theta));
  mat[2][2] =  cos(getRad(dhPar.alpha))*cos(getRad(dhPar.beta));
  mat[3][2] =  0;
  // Matrix m14 bis m44
  mat[0][3] = dhPar.link*cos(getRad(dhPar.theta));
  mat[1][3] = dhPar.link*sin(getRad(dhPar.theta));
  mat[2][3] = dhPar.disp;
  mat[3][3] = 1;

  return mat;
}

/*Returns Linked Transformation-Matrix of Robotic-Arm
  with respect to given Posture */
tmm::Matrix<4,4> Kinematic::armTMatrix(Posture p){
  // D-H Parameter Zuweisen
  dhPar1.theta = p.Jt1;
  dhPar2.theta = p.Jt2;
  dhPar3.theta = p.Jt3;
  dhPar4.theta = p.Jt4;
  dhPar5.theta = p.Jt5;
  dhPar6.theta = p.Jt6;

  //log("===================================================");
  log("D-H Paramter");
  log("dhPar1: " + String(dhPar1.alpha) + " ," + String(dhPar1.link) + ", " + String(dhPar1.disp) + ", " + String(dhPar1.theta));
  log("dhPar2: " + String(dhPar2.alpha) + " ," + String(dhPar2.link) + ", " + String(dhPar2.disp) + ", " + String(dhPar2.theta));
  log("dhPar3: " + String(dhPar3.alpha) + " ," + String(dhPar3.link) + ", " + String(dhPar3.disp) + ", " + String(dhPar3.theta));
  log("dhPar4: " + String(dhPar4.alpha) + " ," + String(dhPar4.link) + ", " + String(dhPar4.disp) + ", " + String(dhPar4.theta));
  log("dhPar5: " + String(dhPar5.alpha) + " ," + String(dhPar5.link) + ", " + String(dhPar5.disp) + ", " + String(dhPar5.theta));
  log("dhPar6: " + String(dhPar6.alpha) + " ," + String(dhPar6.link) + ", " + String(dhPar6.disp) + ", " + String(dhPar6.theta));
  log("===================================================");

  // Achsen-Transformations Matrizen Ermittlen
  tMat01 = jointTMatrix(dhPar1);
  tMat12 = jointTMatrix(dhPar2);
  tMat23 = jointTMatrix(dhPar3);
  tMat34 = jointTMatrix(dhPar4);
  tMat45 = jointTMatrix(dhPar5);
  tMat56 = jointTMatrix(dhPar6);

  // Achsen Transformationen Loggen
  //printMatrix(tMat01, "TMatrix01");
  //printMatrix(tMat12, "TMatrix12");
  //printMatrix(tMat23, "TMatrix23");
  //printMatrix(tMat34, "TMatrix34");
  //printMatrix(tMat45, "TMatrix45");
  //printMatrix(tMat56, "TMatrix56");

  // Verkettet-Transformations Matrizen Ermittlen
  //tMat02 = matrixMulp(tMat01, tMat12);
  //tMat03 = matrixMulp(tMat02, tMat23);
  //tMat04 = matrixMulp(tMat03, tMat34);
  //tMat05 = matrixMulp(tMat04, tMat45);
  //tMat06 = matrixMulp(tMat05, tMat56);


  // Transformationen Loggen
  //printMatrix(tMat02, "TMatrix02");
  //printMatrix(tMat03, "TMatrix03");
  //printMatrix(tMat04, "TMatrix04");
  //printMatrix(tMat05, "TMatrix05");
  //printMatrix(tMat06, "TMatrix06");

  return tMat06;
}

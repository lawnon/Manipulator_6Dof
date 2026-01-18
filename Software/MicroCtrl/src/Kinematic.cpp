
///////////////////////////////////////////////////////////////////////////
/// Datei: Kinematic.cpp
///
/// Beschreibung: Berechnungen und Prozeduren zur Ermittlung der Vor- und
///               Inverskinematik.
///
///               Dieser Klasser die TinyMatrixMath Klasse von "Micah Mundy"
///               https://github.com/m516/TinyMatrixMath?tab=MIT-1-ov-file
///               zu handhaben von Matrizen
///
/// Autor: Chukwunonso Bob-Anyeji
/// Datum: 09.06.2024@12-00
/// Akualisert: 27.08.2025@16-54
///////////////////////////////////////////////////////////////////////////

#include "RobFrameWork.hpp"
#include "Kinematic.hpp"

using namespace Logger;

/// pi Definition
const float pi = 3.1416;

/// Grad zu Radiant konvertieren
float getRad(float deg){
  return deg * (pi/180);
}

/// Radiant zu Grad konvertieren
float getDeg(float rad){
  return rad * (180/pi);
}

/// Signal Wert des Eingangs wiedergeben
float sign(float val){
  return val/abs(val);
}

/// Klassen-Objekt instanzieren und Denavite-Hartenberg Parameter zuweisen
Kinematic::Kinematic(){
  Kinematic::dhPar1 = {90.000, -90.000, 0.000,   0.000, 0.000};
  Kinematic::dhPar2 = { 0.000,   0.000, 0.000, 258.300, 0.000};
  Kinematic::dhPar3 = { 0.000,  90.000, 0.000, 281.516, 0.000};
  Kinematic::dhPar4 = { 0.000, -90.000, 0.000,   0.000, 0.000};
  Kinematic::dhPar5 = { 0.000,  90.000, 0.000,  74.710, 0.000};
  Kinematic::dhPar6 = { 0.000,   0.000, 0.000,   0.000, 0.000};
}

/// Initialisierung Roboter-Framework Zeiger zuweisen
void Kinematic::Init(RobFrame* robFrame){
  _robFrame = robFrame;
}

/// 4x4 Matrix in Seriellen Monitor ausgeben
// TODO: Change String Copy to String Refernece
void Kinematic::printMatrix(tmm::Matrix<4,4> mat, const char* heading){
  log("=============================");
  log(heading);
  mat.printTo(Serial);
}

/// Vorwaertskinematik berechnung anhand der
/// Denavite-Hartenberg Konvention
Position Kinematic::fdKinematic(Posture pt){
  tmm::Matrix<4,4> mat = armTMatrix(pt);
  printMatrix(mat, "Arm Transformation Matrix");

  Position ps {};
  ps.X = mat[0][3];
  ps.Y = mat[1][3];
  ps.Z = mat[2][3];

  ps.B = getDeg(atan2(-mat[2][0], sqrtf(powf(mat[0][0],2) + powf(mat[1][0],2))));
  ps.A = getDeg(atan2(mat[1][0]/cos(getRad(ps.B)), mat[0][0]/cos(getRad(ps.B))));
  ps.C = getDeg(atan2(mat[2][1]/cos(getRad(ps.B)), mat[2][2]/cos(getRad(ps.B))));

  if(ps.B == (getDeg(pi)/2)){
    ps.A = 0;
    ps.C = getDeg(atan2(mat[0][1], mat[1][1]));
  }
  if(ps.B == -(getDeg(pi)/2)) {
    ps.A = 0;
    ps.C = getDeg(-atan2(mat[0][1], mat[2][2]));
  }

  return ps;
}

/// Berechung der Positionsgebenden Winkelstellungen
///  d.h. th1, th2 und th3
Posture Kinematic::ivPosition(Position ps){
  Posture pt = {};
  logln(&ps, "ivPosition: Target Position");

  // Berechnung von theata 1
  float jt1 = atan2(ps.Y, ps.X);
  pt.Jt1 = getDeg(jt1);

  logft(jt1, "jt1");
  logft(pt.Jt1, "pt.jt1");

  // Ermittlung von theta2 durch Anwendung der Linear Kombination von Sinus
  // und Kosinus wellen
  // ie. A*cos(x) + B*sin(x) = C*cos(x + phi) with
  // C = sgn(A).Sqrt(A^2 + B^2) ie. sgn(A) = (A/|A|)
  // phi = arctan(-B/A)
  //
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
  // NOTE: Unused => float jt2_neg = atan2(-sqrt(1-powf(D/C,2)),(D/C)) - Phi;

  pt.Jt2 = getDeg(jt2);

  logft(jt2, "jt2");
  logft(pt.Jt2, "pt.jt2");

  // Berechnung von theta 3:
  float jt3 = atan2(-ps.X*cos(jt1)-ps.Y*sin(jt1)-dhPar2.link*sin(jt2)+dhPar1.link, (ps.Z-dhPar2.link*cos(jt2)))-jt2;
  pt.Jt3 = getDeg(jt3);

  logft(jt3, "jt3");
  logft(pt.Jt3, "pt.jt3");

  return pt;
}

/// 3x3 Rotations-Matrix aus gegebenen 4x4 Transformations-Matrix
/// auslesen.
tmm::Matrix<3,3> Kinematic::read3x3(tmm::Matrix<4,4> tMat){
  tmm::Matrix<3,3> rMat;

  rMat[0][0] = tMat[0][0]; rMat[0][1] = tMat[0][1]; rMat[0][2] = tMat[0][2];
  rMat[1][0] = tMat[1][0]; rMat[1][1] = tMat[1][1]; rMat[1][2] = tMat[1][2];
  rMat[2][0] = tMat[2][0]; rMat[2][1] = tMat[2][1]; rMat[2][2] = tMat[2][2];

  return rMat;
}

/// Ermittlung der Orientierung/Gelenkstellung
/// d.h. th4, th5 und th6
Posture Kinematic::ivOrientation(Posture pt, Position ps){
  // D-H Parameter Zuweisen
  dhPar1.theta = pt.Jt1;
  dhPar2.theta = pt.Jt2;
  dhPar3.theta = pt.Jt3;
  // Achsen-Transformations Matrizen Ermittlen
  tMat01 = jointTMatrix(dhPar1);
  tMat12 = jointTMatrix(dhPar2);
  tMat23 = jointTMatrix(dhPar3);
  // Positins-Transformations Matrizen Ermittlen
  tMat03 = tMat01*tMat12*tMat23;
  tmm::Matrix<3,3> rMat03 = read3x3(tMat03);

  // Ziel Rotations Matrix bauen
  DhParam dhp = {ps.A, ps.B, ps.C,0,0};
  tmm::Matrix<4,4> tMat06 = jointTMatrix(dhp);
  tmm::Matrix<3,3> rMat06 = read3x3(tMat06);

  // Orientierungs Winkeln
  tmm::Matrix<3,3> rMat36 = rMat03.inverse() * rMat06;

  // Berechnung von theta 4
  pt.Jt4 = getDeg(atan2(-rMat36[0][2],rMat36[1][2]));

  // Berechnung von theta 5
  pt.Jt5 = getDeg(atan2(sqrt(powf(rMat36[0][2],2)+powf(rMat36[1][2],2)),rMat36[2][2]));

  // Berechnung von theta 6
  pt.Jt6 = getDeg(atan2(rMat36[2][0],rMat36[2][1]));

  return pt;
}

/// Inverskinematik Berechnund anhand der
/// Denavite-Hartenberge Konvention
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
  pt = ivOrientation(pt, p6);

  // Ermittlung der Positions Difference zwischen Endeffektor und Gelenk 5
  // unterberucksichtigung der Nickung in Acshe 2, 3 oder 5
  if(pt.Jt2 || pt.Jt3 || pt.Jt5){
    // Gelenk 5 entlang der Orientierungs Achse verschieben
    // Matrix4x4  = tmat06v*TransFK(0,90,0,0,0)*TransFK(0,0,0,2*a(5),0);
    //
    // NOTE: Validiere beide Matrizen dhParOpp und tMatDis zusammen
    // getragen werden kann.
    DhParam dhParOpp = {0.000, 90.000, 0.000, 0.000, 0.000};
    DhParam dhParDis = {0.000,  0.000, 0.000, 2*dhPar5.link, 0.000};
    tmm::Matrix<4,4> tMat06 = armTMatrix(pt);
    tmm::Matrix<4,4> tMatOpp = jointTMatrix(dhParOpp);
    tmm::Matrix<4,4> tMatDis = jointTMatrix(dhParDis);

    // Neuer Ziel Position Berechnen
    tmm::Matrix<4,4> tMat06Corr = tMat06*tMatOpp*tMatDis;

    // Berechnung der Positionsgebenden Gelenkstellung d.h. th1, th2 und th3
    // unter Berücksichtigung von Achse 56
    Position p6Corr =
      {tMat06Corr[0][3], tMat06Corr[1][3], tMat06Corr[2][3], p6.A, p6.B, p6.C};
    pt = ivPosition(p6Corr);

    //// Berechnung der Orientierung
    pt = ivOrientation(pt, p6Corr);
  }


  logln(&p6, "ivKinematic: Target Position");
  return pt;
}

/// Homogenen Transformations-Matrix des Gelenks Aufstellen
/// und wiedergeben mit bezug auf die Denavite-Hartenberg Parameter
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
  mat[0][0] = roundf(cos(getRad(dhPar.beta))*cos(getRad(dhPar.theta)));
  mat[1][0] = roundf(cos(getRad(dhPar.beta))*sin(getRad(dhPar.theta)));
  mat[2][0] = roundf(-sin(getRad(dhPar.beta)));
  mat[3][0] = 0;
  // Matrix m12 bis m42
  mat[0][1] = roundf(-cos(getRad(dhPar.alpha))*sin(getRad(dhPar.theta))+sin(getRad(dhPar.alpha))*sin(getRad(dhPar.beta))*cos(getRad(dhPar.theta)));
  mat[1][1] = roundf( cos(getRad(dhPar.alpha))*cos(getRad(dhPar.theta))+sin(getRad(dhPar.alpha))*sin(getRad(dhPar.beta))*sin(getRad(dhPar.theta)));
  mat[2][1] = roundf( sin(getRad(dhPar.alpha))*cos(getRad(dhPar.beta)));
  mat[3][1] =  0;
  // Matrix m13 bis m43
  mat[0][2] = roundf( sin(getRad(dhPar.alpha))*sin(getRad(dhPar.theta))+cos(getRad(dhPar.alpha))*sin(getRad(dhPar.beta))*cos(getRad(dhPar.theta)));
  mat[1][2] = roundf(-sin(getRad(dhPar.alpha))*cos(getRad(dhPar.theta))+cos(getRad(dhPar.alpha))*sin(getRad(dhPar.beta))*sin(getRad(dhPar.theta)));
  mat[2][2] = roundf( cos(getRad(dhPar.alpha))*cos(getRad(dhPar.beta)));
  mat[3][2] =  0;
  // Matrix m14 bis m44
  mat[0][3] = roundf(dhPar.link*cos(getRad(dhPar.theta)));
  mat[1][3] = roundf(dhPar.link*sin(getRad(dhPar.theta)));
  mat[2][3] = roundf(dhPar.disp);
  mat[3][3] = 1;

  return mat;
}

/// Homogenen Transformations-Matrix des Manipulators
/// aufstellen und wiedergeben mit bezug zu Gelenkwinkeln
/// theta 1 bis 6
tmm::Matrix<4,4> Kinematic::armTMatrix(Posture p){
  // D-H Parameter Zuweisen
  dhPar1.theta = p.Jt1;
  dhPar2.theta = p.Jt2;
  dhPar3.theta = p.Jt3;
  dhPar4.theta = p.Jt4;
  dhPar5.theta = p.Jt5;
  dhPar6.theta = p.Jt6;

  /*
  String str1 = "dhPar1: " + String(dhPar1.alpha) + " ," + String(dhPar1.link) + ", " + String(dhPar1.disp) + ", " + String(dhPar1.theta);
  String str2 = "dhPar2: " + String(dhPar2.alpha) + " ," + String(dhPar2.link) + ", " + String(dhPar2.disp) + ", " + String(dhPar2.theta);
  String str3 = "dhPar3: " + String(dhPar3.alpha) + " ," + String(dhPar3.link) + ", " + String(dhPar3.disp) + ", " + String(dhPar3.theta);
  String str4 = "dhPar4: " + String(dhPar4.alpha) + " ," + String(dhPar4.link) + ", " + String(dhPar4.disp) + ", " + String(dhPar4.theta);
  String str5 = "dhPar5: " + String(dhPar5.alpha) + " ," + String(dhPar5.link) + ", " + String(dhPar5.disp) + ", " + String(dhPar5.theta);
  String str6 = "dhPar6: " + String(dhPar6.alpha) + " ," + String(dhPar6.link) + ", " + String(dhPar6.disp) + ", " + String(dhPar6.theta);
  log("===================================================");
  log("D-H Paramter");
  log(str1.c_str());
  log(str2.c_str());
  log(str3.c_str());
  log(str4.c_str());
  log(str5.c_str());
  log(str6.c_str());
  log("===================================================");

  log("===================================================");
  log("D-H Paramter");
  log("dhPar1: " + String(dhPar1.alpha) + " ," + String(dhPar1.link) + ", " + String(dhPar1.disp) + ", " + String(dhPar1.theta));
  log("dhPar2: " + String(dhPar2.alpha) + " ," + String(dhPar2.link) + ", " + String(dhPar2.disp) + ", " + String(dhPar2.theta));
  log("dhPar3: " + String(dhPar3.alpha) + " ," + String(dhPar3.link) + ", " + String(dhPar3.disp) + ", " + String(dhPar3.theta));
  log("dhPar4: " + String(dhPar4.alpha) + " ," + String(dhPar4.link) + ", " + String(dhPar4.disp) + ", " + String(dhPar4.theta));
  log("dhPar5: " + String(dhPar5.alpha) + " ," + String(dhPar5.link) + ", " + String(dhPar5.disp) + ", " + String(dhPar5.theta));
  log("dhPar6: " + String(dhPar6.alpha) + " ," + String(dhPar6.link) + ", " + String(dhPar6.disp) + ", " + String(dhPar6.theta));
  log("===================================================");

   */


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
  //printMatrix(tMat34, "TMatrix34");
  //printMatrix(tMat45, "TMatrix45");
  //printMatrix(tMat56, "TMatrix56");
  //printMatrix(tMat23, "TMatrix23");

  // Verkettet-Transformations Matrizen Ermittlen
  tMat02 = tMat01*tMat12;
  tMat03 = tMat02*tMat23;
  tMat04 = tMat03*tMat34;
  tMat05 = tMat04*tMat45;
  tMat06 = tMat05*tMat56;


  // Transformationen Loggen
  //printMatrix(tMat02, "TMatrix02");
  //printMatrix(tMat03, "TMatrix03");
  //printMatrix(tMat04, "TMatrix04");
  //printMatrix(tMat05, "TMatrix05");
  //printMatrix(tMat06, "TMatrix06");

  return tMat06;
}

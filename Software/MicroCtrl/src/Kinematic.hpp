
//////////////////////////////////////////////////////////////////////////
// Datei: Kinematic.hpp
//
// Beschreibung: Berechnungen und Prozeduren zur Ermittlung der Vor- und
//               Inverskinematik.
//
//               Dieser Klasser die TinyMatrixMath Klasse von "Micah Mundy"
//               https://github.com/m516/TinyMatrixMath?tab=MIT-1-ov-file
//               zu handhaben von Matrizen
//
// Autor: Chukwunonso Bob-Anyeji
// Datum: 09.06.2024@12-00
// Akualisert: 27.08.2025@16-54
//////////////////////////////////////////////////////////////////////////

#ifndef KINEMATIC_H_
#define KINEMATIC_H_

#include <math.h>
#include <WString.h>
#include <HardwareSerial.h>
#include <TinyMatrixMath.hpp>

#include "Types.hpp"
#include "Logger.hpp"

/// @brief Grad zu Radiant
/// @param deg Grad wert
/// @return Radiant
float getRad(float deg);

/// @brief Radiant zu Grad
/// @param rad Radiant
/// @return Grad
float getDeg(float rad);

/// @brief Signum Funktion
/// @param value
/// @return 1 oder -1
float sign(float value);

class RobFrame;
class Kinematic
{
   private:
        /// Zeiger zu RobFrame
        RobFrame* _robFrame;
        /// @brief Homogenen Transformations-Matrix des gelenks
        /// @param dhPar Denavite-Hartenberge Parameter
        /// @return Transformations-Matrix
        tmm::Matrix<4,4> jointTMatrix(DhParam dhPar);
        /// @brief Homogenen Transformations-Matrix des Manipulators
        /// @param Manipulator-Stellung Theta 1 bis 6
        /// @return Transformations-Matrix
        tmm::Matrix<4,4> armTMatrix(Posture posture);
        /// @brief Rotations-Matrix auslesen
        /// @param tMat Transformations-Matrix
        tmm::Matrix<3,3> read3x3(tmm::Matrix<4,4> tMat);
    public:
        /// @brief Kinematic Klassen-Objekt
        Kinematic();
        /// @brief Initialisierung Zeiger zuweisung
        /// @param robFrame Zeiger zu RobFrame
        /// @return void
        void Init(RobFrame* robFrame);
        /// @brief Matrix drucken
        /// @param mat 4x4 Matrix
        /// @param heading Matrix Tag
        /// @return void
        void printMatrix(tmm::Matrix<4,4> mat, String heading);
        /// @brief Vorwaertskinematik berechnung
        /// @param posture Manipulator Winkelstellung
        /// @return Lage (Position und Orientierung)
        Position fdKinematic(Posture posture);
        /// @brief Inverskinematik berechnung
        /// @param position Manipulator Position
        /// @return Manipulator Winkelstellung
        Posture ivKinematic(Position position);
        /// @brief Positionsgebenden Winkeln Berechnen
        /// @param position lage Position und Orientierune
        /// @return Winkelwerte 1...3
        Posture ivPosition(Position position);
        /// @brief Orientierungsgebeden Winkeln Berechnen
        /// @param posture Winkelstellung
        /// @param Ps Position
        /// @return Winkelstellung
        Posture ivOrientation(Posture posture, Position ps);

        /// @brief D-H Parameter Gelenk 1
        DhParam dhPar1 = {};
        /// @brief D-H Parameter Gelenk 2
        DhParam dhPar2 = {};
        /// @brief D-H Parameter Gelenk 3
        DhParam dhPar3 = {};
        /// @brief D-H Parameter Gelenk 4
        DhParam dhPar4 = {};
        /// @brief D-H Parameter Gelenk 5
        DhParam dhPar5 = {};
        /// @brief D-H Parameter Gelenk 6
        DhParam dhPar6 = {};

        /// @brief Achsen-Transformations-Matrix 1
        tmm::Matrix<4,4> tMat01 = {};
        /// @brief Achsen-Transformations-Matrix 2
        tmm::Matrix<4,4> tMat12 = {};
        /// @brief Achsen-Transformations-Matrix 3
        tmm::Matrix<4,4> tMat23 = {};
        /// @brief Achsen-Transformations-Matrix 4
        tmm::Matrix<4,4> tMat34 = {};
        /// @brief Achsen-Transformations-Matrix 5
        tmm::Matrix<4,4> tMat45 = {};
        /// @brief Achsen-Transformations-Matrix 6
        tmm::Matrix<4,4> tMat56 = {};

        /// @brief Verkettet Transformations-Matrix 02
        tmm::Matrix<4,4> tMat02 = {};
        /// @brief Verkettet Transformations-Matrix 03
        tmm::Matrix<4,4> tMat03 = {};
        /// @brief Verkettet Transformations-Matrix 04
        tmm::Matrix<4,4> tMat04 = {};
        /// @brief Verkettet Transformations-Matrix 05
        tmm::Matrix<4,4> tMat05 = {};
        /// @brief Verkettet Transformations-Matrix 06
        tmm::Matrix<4,4> tMat06 = {};
};

#endif // KINEMATIC_H_

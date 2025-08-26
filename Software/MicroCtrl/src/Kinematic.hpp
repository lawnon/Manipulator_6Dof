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
#include <TinyMatrixMath.hpp>

#include "Types.hpp"
#include "Logger.hpp"


// Misc
float getRad(float deg);
float getDeg(float rad);
float sign(float);

class RobFrame;
class Kinematic
{
    private:
        RobFrame* _robFrame;

        Matrix4x4 matrixMulp(Matrix4x4 A, Matrix4x4 B);
        tmm::Matrix<4,4> jointTMatrix(DhParam dhPar);
        tmm::Matrix<4,4> armTMatrix(Posture posture);
    public:
        Kinematic();

        void Init(RobFrame* robFrame);
        void printMatrix(Matrix4x4 mat, String heading);
        Position fdKinematic(Posture posture);
        Posture ivKinematic(Position position);
        Posture ivPosition(Position position);
        Posture ivOrientation(Posture posture, Position ps);

        // Danavit Hartenberg Parameter
        DhParam dhPar1 = {};
        DhParam dhPar2 = {};
        DhParam dhPar3 = {};
        DhParam dhPar4 = {};
        DhParam dhPar5 = {};
        DhParam dhPar6 = {};

        // Achsen-Transformations Matrizen
        tmm::Matrix<4,4> tMat01 = {};
        tmm::Matrix<4,4> tMat12 = {};
        tmm::Matrix<4,4> tMat23 = {};
        tmm::Matrix<4,4> tMat34 = {};
        tmm::Matrix<4,4> tMat45 = {};
        tmm::Matrix<4,4> tMat56 = {};
        // Verkettet Transformations Matrizen
        tmm::Matrix<4,4> tMat02 = {};
        tmm::Matrix<4,4> tMat03 = {};
        tmm::Matrix<4,4> tMat04 = {};
        tmm::Matrix<4,4> tMat05 = {};
        tmm::Matrix<4,4> tMat06 = {};
};

#endif // KINEMATIC_H_

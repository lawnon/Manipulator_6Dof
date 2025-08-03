%% Datei: VrwKinematik.m
% Beschreibung: Evaluation und Emittlung der Forwärts-Kinematik
% Autor: Chukwunonso Bob-Anyeji
% Datum: 27-07-2025@11-52
%=========================================================================
function [A1_2, A1_3, A1_4, A1_5, A1_6, A1_E, Rvec] = VrwKinematik(t1, t2, t3, t4, t5, t6, cl, zv, path)
clc
% Ermittlung der Manipulator Transformations-Matrix 
[A1_2, A1_3, A1_4, A1_5, A1_6, A1_E] = ArmTrans(t1, t2, t3, t4, t5, t6);
%
% Evaluation des Rotations Vectors
Rot = zeros(1,3);
if (A1_E(1,1) == 0 && A1_E(2,1) == 0)
    Rot(1,1) = rad2deg(atan2(A1_E(1,2), A1_E(2,2)));
    Rot(1,2) = rad2deg(0);
    Rot(1,3) = rad2deg(0);
else
    Rot(1,1) = rad2deg(atan2(A1_E(3,2), A1_E(3,3)));
    Rot(1,2) = rad2deg(atan2(-A1_E(3,1), sqrt(A1_E(1,1)^2 + A1_E(2,1)^2)));
    Rot(1,3) = rad2deg(atan2(A1_E(2,1), A1_E(1,1)));
end
Rvec = Rot;
%
%% Plot Graphs
clf
%[1] Plot Posture of Robot
subplot(2,2,1);
PlotArmTrans(A1_2, A1_3, A1_4, A1_5, A1_6, A1_E, cl, zv, 0, 0, path);
%[2] Plot Posture of Robot
subplot(2,2,2);
PlotArmTrans(A1_2, A1_3, A1_4, A1_5, A1_6, A1_E, cl, zv, 90, 0, path);
%[3] Plot Posture of Robot
subplot(2,2,[3,4]);
PlotArmTrans(A1_2, A1_3, A1_4, A1_5, A1_6, A1_E, cl, zv, 45, 45, path);
end
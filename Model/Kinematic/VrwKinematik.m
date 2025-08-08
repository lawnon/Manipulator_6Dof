%% Datei: VrwKinematik.m
% Beschreibung: Evaluation und Emittlung der Forwärts-Kinematik
% Autor: Chukwunonso Bob-Anyeji
% Datum: 27-07-2025@11-52
%=========================================================================
function [Pos] = VrwKinematik(th1, th2, th3, th4, th5, th6, cl, zv, path)
clc
% Ermittlung der Manipulator Transformations-Matrix 
TMat = ArmTrans(th1, th2, th3, th4, th5, th6);

% Evaluation der Winkelwerte
Pos = zeros(6,1);
%Pos(1:3) = TMat(6)(1:3,4);
%b = atan2(-TMat(6)(3,1), sqrt((TMat(6)(1,1))^2 + (TMat(6)(2,1)^2)));

if (A1_E(1,1) == 0 && A1_E(2,1) == 0)
    Pos(1,4) = rad2deg(atan2(A1_E(1,2), A1_E(2,2)));
    Pos(1,5) = rad2deg(pi/2);
    Pos(1,6) = rad2deg(0);
else
    Pos(1,4) = rad2deg(atan2(A1_E(3,2), A1_E(3,3)));
    Pos(1,5) = rad2deg(atan2(-A1_E(3,1), sqrt(A1_E(1,1)^2 + A1_E(2,1)^2)));
    Pos(1,6) = rad2deg(atan2(A1_E(2,1), A1_E(1,1)));
end

% Plot Graphs
clf
%[1] Plot Posture of Robot
subplot(2,2,1);
PlotArmTrans(A1_2, A1_3, A1_4, A1_5, A1_6, A1_E, cl, zv, 90, 0, path);
%[2] Plot Posture of Robot
subplot(2,2,2);
PlotArmTrans(A1_2, A1_3, A1_4, A1_5, A1_6, A1_E, cl, zv, 0, 0, path);
%[3] Plot Posture of Robot
subplot(2,2,3);
PlotArmTrans(A1_2, A1_3, A1_4, A1_5, A1_6, A1_E, cl, zv, 90, 90, path);
%[4] Plot Posture of Robot
subplot(2,2,4);
PlotArmTrans(A1_2, A1_3, A1_4, A1_5, A1_6, A1_E, cl, zv, 45, 45, path);
end
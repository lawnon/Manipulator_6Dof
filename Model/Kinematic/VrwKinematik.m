%% Datei: VrwKinematik.m
% Beschreibung: Evaluation und Emittlung der Forwärts-Kinematik
% Autor: Chukwunonso Bob-Anyeji
% Datum: 27-07-2025@11-52
%=========================================================================
function [Pos,Pos5,TMat0_6,RMat] = VrwKinematik(...
    th1,th2,th3,th4,th5,th6,cl,zv,sz,plot)
% Ermittlung der Manipulator Transformations-Matrix
[TMat0_1, TMat0_2, TMat0_3, TMat0_4, TMat0_5, TMat0_6] =...
    ArmTrans(th1, th2, th3, th4, th5, th6)

% Zuweistung der Manipulator Position und Orientierung
Pos5 = TMat0_4(1:3,4);
RMat = TMat0_6(1:3,1:3);

% Winkelwerte Auswerten
[alpha, beta, gamma] = ToAngles(TMat0_6,3);

% Positions Werte Widergeben
Pos = zeros(6,1);
Pos(1:3) = round(TMat0_6(1:3,4),3);
Pos(4)   = ToDeg(round(alpha,3));   %Yaw
Pos(5)   = ToDeg(round(beta,3));    %Pitch
Pos(6)   = ToDeg(round(gamma,3));   %Roll

switch plot
    case 0
        return;
    case 1
        PlotArmTrans(TMat0_1,TMat0_2,TMat0_3,TMat0_4,TMat0_5,TMat0_6...
            ,cl, zv, 45, 45, sz, true);
    case 2
        
        % Plot Graphs
        %[1] Plot Posture of Robot
        subplot(2,2,1);
        PlotArmTrans(TMat0_1, TMat0_2, TMat0_3, TMat0_4, TMat0_5, TMat0_6,...
            cl, zv, 90, 0, sz, true);
        %[2] Plot Posture of Robot
        subplot(2,2,2);
        PlotArmTrans(TMat0_1, TMat0_2, TMat0_3, TMat0_4, TMat0_5, TMat0_6,...
            cl, zv, 0, 0, sz, true);
        %[3] Plot Posture of Robot
        subplot(2,2,3);
        PlotArmTrans(TMat0_1, TMat0_2, TMat0_3, TMat0_4, TMat0_5, TMat0_6,...
            cl, zv, 90, 90, sz, true);
        %[4] Plot Posture of Robot
        subplot(2,2,4);
        PlotArmTrans(TMat0_1, TMat0_2, TMat0_3, TMat0_4, TMat0_5, TMat0_6,...
            cl, zv, 45, 45, sz, true);
end
end
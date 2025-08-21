%% Matlab Script zur Masterarbeit: 
% Datei: 6DofArm_Kinematik.m
% Beschreibung: Konstruktion, Modellierung und Programmierung der 
%               Inversen Kinematik eines experimentellen Manipulators 
%               mit 6 Freiheitsgraden
% Autor: Chukwunonso Bob-Anyeji
% Datum: 27-07-2025@11-52
% =========================================================================
%

%% Auswertung der Vorwärtskinematik:
clear; clc; clf;

% Denavite und Hartenberg Parametersatz Wählen
dhp = 1;
plot = 1;

% Vorwärtskinematik

[PosVk,Pos5Vk, TMat06v,RMatv] = VrwKinematik(0,-90,90,45,-90,-45,[0 0.4470 0.7410],'-o',10,plot,dhp);
%[PosVk,Pos5Vk, TMat06v,RMatv] = VrwKinematik(0,0,0,0,0,0,[0 0.4470 0.7410],'-o',10,plot,dhp);

%% Auswertung der InverseKinematik:
clear; clc; clf;

% Denavite und Hartenberg Parametersatz Wählen
dhp = 1;
elbow = 1;
plot = 1;

% Rotations(Yaw,Pitch,Roll) Vorgabe
Rot = [0;0;0];
rmat = Rotation(Rot(1),Rot(2),Rot(3)); 

% Inverse Kinematik 
Pos = [300;300;300;Rot(1);Rot(2);Rot(3)];
[td1,td2,td3,td4,td5,td6] = InvKinematik(Pos(1),Pos(2),Pos(3),rmat,dhp,elbow); 
Angles = [td1;td2;td3;td4;td5;td6]

% Validierung durch Vorwärtskinematik
[PosIk,Pos5Ik,TMat06v,RMati] = VrwKinematik(...
    td1,td2,td3,td4,td5,td6,[0.3010 0.7450 0.9330],'-x',10,plot,dhp);

% Positions Difference Herausrechnen und mit loggen
diff = abs(Pos-PosIk);
Log = [Pos PosIk diff Angles];

%% Validierung der InverseKinematik anhand der Vorwärts-Kinemati:
clear; clc; clf;

% Denavite und Hartenberg Parametersatz Wählen
dhp = 1;
elbow = 1;
plot = 2;

% Vorwärts Kinematik
[PosVk,Pos5Vk, TMat06v,RMatv] = VrwKinematik(...
    60,100,-80,45,70,-220,[0 0.4470 0.7410], '-+',10,plot,dhp);

% Inverse Kinematik
[td1,td2,td3,td4,td5,td6] = InvKinematik(...
    PosVk(1),PosVk(2),PosVk(3),RMatv,dhp,elbow);
Angles = [td1;td2;td3;td4;td5;td6]

% Validierung durch Vorwärtskinematik
[PosIk,Pos5Ik,TMat06v,RMati] = VrwKinematik(...
    td1,td2,td3,td4,td5,td6,[0.3010 0.7450 0.9330],'-x',10,plot,dhp);

% Positions Difference Herausrechnen und mit loggen
diff = abs(PosVk-PosIk);
Log = [PosVk PosIk diff Angles];

%% Validierung der InverseKinematik anhand der Vorwärts-Kinemati:
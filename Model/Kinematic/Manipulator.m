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
%[PosVk,Pos5Vk, TMat06v,RMatv] = VrwKinematik(...
%   60, 100, -80, 45, 70, -220, [0 0.4470 0.7410], '-+', true)
[PosVk,Pos5Vk, TMat06v,RMatv] = VrwKinematik(...
    26,63,-73,45,-5,60,[0 0.4470 0.7410],'-+',10,true);

%% Auswertung der InverseKinematik:
% [td1,td2,td3,td4,td5,td6] = InvKinematik(...
%   Pos5Vk(1),Pos5Vk(2),Pos5Vk(3),RMatv);
rmat = Rotation(-45,45,55); % Rotation(Yaw,Pitch,Roll)
[td1,td2,td3,td4,td5,td6] = InvKinematik(-250,-255,250,rmat)
[PosIk,Pos5Ik,TMat06v,RMati] = VrwKinematik(...
    td1,td2,td3,td4,td5,td6,[0.3010 0.7450 0.9330],'-x',10,2);

%% Hilfsfunktionene 
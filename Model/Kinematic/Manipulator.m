%% Matlab Script zur Masterarbeit: 
% Datei: 6DofArm_Kinematik.m
% Beschreibung: Konstruktion, Modellierung und Programmierung der 
%               Inversen Kinematik eines experimentellen Manipulators 
%               mit 6 Freiheitsgraden
% Autor: Chukwunonso Bob-Anyeji
% Datum: 27-07-2025@11-52
%=========================================================================
%
%% Auswertung der Vorwärtskinematik:
clear
%Pos = VrwKinematik(30, 100, -60, -90, -90, 0, 'b', '-+', true)
Pos = VrwKinematik(0, 0, 0, 0, 0, 0, 'b', '-+', true)
PlotEndPoint(Pos, 'r', '-o');

%% Auswertung der InverseKinematik:
%p = [0, -400, 300, 0, 0, 0];
Pos(4:6) = [0,0,0]
[td1,td2,td3,td4,td5,td6] = InvKinematik(Pos)
VrwKinematik(td1,td2,td3,td4,td5,td6,'b','-+',true);
PlotEndPoint(Pos, 'r', '-o');

%% Hilfsfunktionene 
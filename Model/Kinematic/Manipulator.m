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
VrwKinematik(30, 100, -60, -90, -90, 0, 'b', '-+', true);

%% Auswertung der InverseKinematik:
clear
[td1,td2,td3] = InvKinematik(0, 400, 300, 0, 0, 0)
VrwKinematik(td1,td2,td3,0,0,0,'b','-+',true);
%InvKinematik(x,y,z,rx,ry,rz)
%% Hilfsfunktionene 

%Static Load Analysis of 6 DOF Arm for Surpport in Chosing suitable Motors
%and Mechanical Construction of the Robotic Arm

clear;
clc;

%Note: Dimenstions in Standard SI-Units ie Kg, m, s and 

g = 9.81; %[m/s^2]

%Moments
Mjt2 = 8;

%Joint Mass ie. combined mass of Joints and Motors in [Kg]
mjt01 = 1.7286;
mjt12 = 1.5004;
mjt23 = 0.9747;
mjt34 = 0.4649;
mjt45 = 0.6774;
mjt56 = 0.1413;
mjt6E = 0.0832;

%Center of Gravity with respect or y Axis in [m]
rjt01 = -0.001;
rjt02 = -0.001;
rjt03 = 0.187;
rjt04 = 0.324;
rjt05 = 0.459;
rjt06 = 0.545;
rjt0E = 0.603;

%Resulting Moment on each Joint with respect to Joint 1
%Moment = Force x Penpedicular Distance ie. Mass x Gravity x Distance
Mjt12 = mjt12 * g * abs(rjt02);
Mjt13 = mjt23 * g * rjt03;
Mjt14 = mjt34 * g * rjt04;
Mjt15 = mjt45 * g * rjt05;
Mjt16 = mjt56 * g * rjt06;
Mjt1E = mjt6E * g * rjt0E;
%Total Moment of Force from Joint 2 to Endeffector
Mjt1E_t = Mjt12 + Mjt13 + Mjt14 + Mjt15 + Mjt16 + Mjt1E

%Resulting Moment on each Joint with respect to Joint 2
Mjt23 = mjt23 * g * rjt03;
Mjt24 = mjt34 * g * rjt04;
Mjt25 = mjt45 * g * rjt05;
Mjt26 = mjt56 * g * rjt06;
Mjt2E = mjt6E * g * rjt0E;
%Total Moment of Force from Joint 2 to Endeffector
Mjt2E_t = Mjt23 + Mjt24 + Mjt25 + Mjt26 + Mjt2E

%Resulting Moment on each Joint with respect to Joint 3
% Offset from Zero 
rjt32 = 0.2515;
Mjt34 = mjt34 * g * (rjt04 - rjt32);
Mjt35 = mjt45 * g * (rjt05 - rjt32);
Mjt36 = mjt56 * g * (rjt06 - rjt32);
Mjt3E = mjt6E * g * (rjt0E - rjt32);
%Total Moment of Force from Joint 2 to Endeffector
Mjt3E_t = Mjt34 + Mjt35 + Mjt36 + Mjt3E

%Resulting Moment on each Joint with respect to Joint 4
% Offset from Zero 
rjt42 = 0.3665;
Mjt45 = mjt45 * g * (rjt05 - rjt42);
Mjt46 = mjt56 * g * (rjt06 - rjt42);
Mjt4E = mjt6E * g * (rjt0E - rjt42);
%Total Moment of Force from Joint 2 to Endeffector
Mjt4E_t = Mjt45 + Mjt46 + Mjt4E

%Resulting Moment on each Joint with respect to Joint 5
% Offset from Zero 
rjt52 = 0.4871;
Mjt56 = mjt56 * g * (rjt06 - rjt52);
Mjt5E = mjt6E * g * (rjt0E - rjt52);
%Total Moment of Force from Joint 2 to Endeffector
Mjt5E_t = Mjt56 + Mjt5E

%Resulting Moment on each Joint with respect to Joint 6
% Offset from Zero 
rjt62 = 0.6019;
Mjt6E = mjt6E * g * (rjt0E - rjt62);
%Total Moment of Force from Joint 2 to Endeffector
Mjt6E_t = Mjt6E



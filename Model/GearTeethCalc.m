clear;
clc;
% Determination of Gear Diameter and Teeth Count with Repect of to Teeteh
% Period.
T_p = 6.8068; %Teeth Period

%Estimate Teeth Number
r_est = 18;
U_est = 2*pi*r_est;
T_nr = U_est/T_p;
T_nrf = ceil(T_nr)
U = T_nrf * T_p;
r = U/(2*pi)

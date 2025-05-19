function [V_ref] = GetVref(I_limit)
%Calculate Current Limit of AZ- A4988 Stepper Motor Controller
% Current Limit = V-Reference * R-Board
% I_lim[Ampere] = V_ref[Volt] * R_bd[Ohm]
R_bd = 2.5;

% Considering 70% Power Output Efficiency
% Target Output * 70% = Actual Output ie.
I_lim_act = I_limit / 0.7;

V_ref = I_lim_act / R_bd;
end


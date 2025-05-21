clear;
clc;
%Electrical Calculations

% Reference Voltage for Current Limit on an AZ- A4988 Stepper Motor Controller
%Current Limit for 28BYJ-48 – 5V Stepper Motor
Vref_28BYJ_48 = GetVref(0.7)
%Current Limit for Sumtor 42HS4013A4-G10
Vref_Sumtor_42HS4013A4 = GetVref(1.3)
%Current Limit for Wantmotor 42BYGHW811
Vref_Wantai_42BYGHW811_X3 = GetVref(2.5)
%Current Limit for Wantmotor 2BYGHW609L20P1-X2
Vref_Wantai_2BYGHW609L20P1_X2 = GetVref(1.7)
%Current Limit for Stepper Online 17HS19-1684S-PG14
Vref_Wantai_17HS19_1684S_PG14 = GetVref(1.68)


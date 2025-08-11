%% Datei: Invkinematik.m
% Beschreibung: Evaluation und Emittlung der Inverse-Kinematik
% Autor: Chukwunonso Bob-Anp(2)eji
% Datum: 27-07-2025@11-52
%
function [th1,th2,th3,th4,th5,th6] = InvKinematik(x, y, z, roll, pitch, yaw)
% DH Parameter auslesen
[alpha, beta, a, d] =  DhParams();

% Berechnung der Positionsgebenden Gelenkstellung d.h. th1, th2 und th3
% 
% TMat = |RotationsMatrix(3x3) Verschiebung(3,1)|
%        |      0                     1         |
%        
% TMat03 = TMat1*TMat2*TMat3 
% TMat03 =...
%     |-sin(th1) -cos(th1)*sin(th2+th3) cos(th1)*cos(th2+th3) cos(th1)*(a(3)*cos(th2+th3)+a(2)*cos(th2)+a(1))|
%     | cos(th1) -sin(th1)*sin(th2+th3) sin(th1)*cos(th2+th3) sin(th1)*(a(3)*cos(th2+th3)+a(2)*cos(th2)+a(1))|
%     |    0          cos(th2+th3)           cos(th2+th3)            a(3)*sin(th2+th3)+a(2)*sin(th2)         |
%     |    0              0                      0                                  1                        |
%                     
[thr1,thr2,thr3] = Position(x,y,z,3);
th1 = ToDeg(thr1);
th2 = ToDeg(thr2);
th3 = ToDeg(thr3);

% Berechnung der Orientierungsgebenden Gelenkstellung d.h. th4, th5 und th6
% TMat0_6 = TMat0_3 * TMat4_6
% TMat0_6 * (TMat0_3)^-1 = TMat0_3 * TMat4_6 * (TMat0_3)^-1
%
% Associativitaet
% TMat0_6 * (TMat0_3)^-1 = TMat4_6 * TMat0_3 * (TMat0_3)^-1
%
% d.h.
% TMat4_6 = TMat0_6 * (TMat0_3)^-1
%
% mit TMat0_6 = |RotationsMatrix(3x3) Verschiebung(3x1)|
%               |      0(1x3)                1         |

% Ermittlung der Manipulator Transformations-Matrix entsprechend der
% gegebenen Positions-Winkelwerten
[tmat0_1,tmat0_2,tmat0_3] = ArmTrans(th1,th2,th3,0,0,0);

tmat0_6 = eye(4,4)
tmat0_6 = Trans(tmat0_6,x,y,z,roll,pitch,yaw);
tmat4_6 = tmat0_6 * (tmat0_3)^-1;

% Berechnung der Orientierung
[thr4,thr5,thr6] = Orientation(tmat4_6, 3);
th4 = ToDeg(thr4);
th5 = ToDeg(thr5);
th6 = ToDeg(thr6);

% Ermittlung der Positions Difference zwischen Endeffektor und Gelenk 5
% unterberucksichtigung der Nickung in Acshe 2, 3 oder 5
if (th2 ~= 0 || th3 ~= 0 || th5 ~= 0)
    % Positions Difference berechnen
    [posVk, tmat06v] = VrwKinematik(th1,th2,th3,th4,th5,th6,'r','-+',1);
    
    % Gelenk 5 entlang der Orientierungs Achse verschieben
    tmatd = tmat06v*TransFK(0,90,0,0,0)*TransFK(0,0,0,2*a(5),0);
    PlotEndPoint(tmatd, 50, '-o');
    
    % Berechnung der Positionsgebenden Gelenkstellung d.h. th1, th2 und th3
    % unter Berücksichtigung von Achse 56    
    [thr1,thr2,thr3] = Position(tmatd(1,4),tmatd(2,4),tmatd(3,4),3);
    th1 = ToDeg(thr1);
    th2 = ToDeg(thr2);
    th3 = ToDeg(thr3);
    
    % Ermittlung der Manipulator Transformations-Matrix entsprechend der
    % gegebenen Positions-Winkelwerten
    [tmat0_1,tmat0_2,tmat0_3] = ArmTrans(th1,th2,th3,0,0,0);
    
    tmat0_6 = eye(4,4)
    tmat0_6 = Trans(tmat0_6,x,y,z,roll,pitch,yaw);
    tmat4_6 = tmat0_6 * (tmat0_3)^-1;
    
    % Ermittlung der Manipulator Orientierung
    %[thr4,thr5,thr6] = Orientation(tmat4_6, 3);
    [thr4,thr5,thr6] = Orientation(tmat06v,3);
    th4 = ToDeg(thr4);
    th5 = ToDeg(thr5);
    th6 = ToDeg(thr6);
end
end
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

% Berechnung von theta 1
thr1 = atan2(y,x);
thr1 = real(thr1);
thr1 = round(thr1,3);
th1 = ToDeg(thr1);

% Berechnung von theta 2
% Ermittlung von theta2 durch Anwendung der Linear Kombination von Sinus
% und Kosinus wellen
% ie. A*cos(x) + B*sin(x) = C*cos(x + phi) with
% C = sgn(A).Sqrt(A^2 + B^2) ie. sgn(A) = (A/|A|)
% phi = arctan(-B/A)

A = -z;
B = a(1)-(x*cos(thr1)+y*sin(thr1));
Dn = 2*a(1)*(x*cos(thr1)+y*sin(thr1))+(a(3)^2)-(a(2)^2)-(a(1)^2)-(z^2)-(x*cos(thr1)+y*sin(thr1))^2;
Dd = 2*a(2);
D = Dn/Dd;
%
phi = real(atan2(B,A));
r = sqrt(A^2+B^2); 
%
rD = [real(sqrt(r^2-D^2));real(-sqrt(r^2-D^2))];
% Insert Geometrie(2) Constriant
% for now use Positive Value
thr2 = -phi+atan2(D,rD(1));
thr2 = real(thr2);
thr2 = round(thr2,3);
th2 = ToDeg(thr2);

% Berechnung von theta 3:
thr3 = atan2(z-a(2)*sin(thr2), x*cos(thr1)+y*sin(thr1)-a(2)*cos(thr2)-a(1))-thr2;
thr3 = real(thr3);
thr3 = round(thr3,3);
th3 = ToDeg(thr3);

[th1,th2,th3] = PosAngles(x,y,z);

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
% gegebene Ziel Position & Orientierung
[tmat0_1, tmat0_2, tmat0_3] = ArmTrans(th1, th2, th3, 0, 0, 0);

tmat0_6 = eye(4,4)
tmat0_6 = Trans(tmat0_6,x,y,z,roll,pitch,yaw);
tmat4_6 = tmat0_6 * (tmat0_3)^-1;

% Berechnung von theta 5
thr5 = acos(tmat4_6(3,3));
thr5 = real(thr5);
thr5 = round(thr5,3);
th5 = ToDeg(thr5);

% Berechnung von theta 6
thr6 = asin(tmat4_6(3,1)/sin(thr5));
thr6 = real(thr6);
thr6 = round(thr6,3);
th6 = ToDeg(thr6);

% Berechnung von theta 4
thr4 = acos(tmat4_6(2,3)/sin(thr5));
thr4 = real(thr4);
thr4 = round(thr4,3);
th4 = ToDeg(thr4);

% Positions Difference berechnen
[posVk, tmat06v] = VrwKinematik(th1,th2,th3,th4,th5,th6,[0 0.4470 0.7410],'-+',0);
posdiff = posVk(1:3) - [x;y;z];

end
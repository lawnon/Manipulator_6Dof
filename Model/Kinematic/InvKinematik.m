%% Datei: Invkinematik.m
% Beschreibung: Evaluation und Emittlung der Inverse-Kinematik
% Autor: Chukwunonso Bob-Anp(2)eji
% Datum: 27-07-2025@11-52
%
function [td1,td2,td3,td4,td5,td6] = InvKinematik(Pend)
clc
%% DH Parameter auslesen
[alpha beta a d] =  DhParams();

%% Berechnung der Positionsgebenden Gelenkstellung d.h. th1, th2 und th3
% 
% TMat = |RotationsMatrip(1)(3p(1)3) Verschiebung(3p(1)1)|
%        |      0(1p(1)3)                1         |
%        
% A1_4 = A1_2 * A2_3 * A3_4
% A1_4 = |cos(th1)*cos(th2+th3) -sin(th2+th3)*cos(th1)  sin(th1) cos(th1)*(a(3)*cos(th2+th3)+a(2)*cos(th2)+a(1))|
%        |sin(th1)*cos(th2+th3) -sin(th2+th3)*sin(th1) -cos(th1) sin(th1)*(a(3)*cos(th2+th3)+a(2)*cos(th2)+a(1))|
%        |    sin(th2+th3)           cos(th2+th3)          0            a(3)*sin(th2+th3)+a(2)*sin(th2)         |
%        |         0                      0                0                           1                        |
%                     

% Berechnung der Gelenk 5 Verschiebung
p = zeros(1,6);

p(1) = Pend(1) - a(5)*cos(deg2rad(Pend(5)));
%p(3) = Pend(3) - a(5)*sin(deg2rad(Pend(5)));
opp56 = a(5)*sin(deg2rad(Pend(5)));
p(2) = Pend(2) + opp56*sin(deg2rad(Pend(4)));
p(3) = Pend(3) - opp56*cos(deg2rad(Pend(4)));
%p(1) = Pend(1) - a(5)*cos(deg2rad(Pend(5)));
%p(2) = Pend(2) - a(5)*cos(deg2rad(Pend(6)));
%p(3) = Pend(3) - a(5)*sin(deg2rad(Pend(5)));

% Berechnung von theta 1: 
% p(1) = cos(th1)*(a(3)*cos(th2+th3)+a(2)*cos(th2)+a(1))
% p(2) = sin(th1)*(a(3)*cos(th2+th3)+a(2)*cos(th2)+a(1))
% p(3) = a(3)*sin(th2+th3)+a(2)*sin(th2)
tr1 = round(atan2(p(2),p(1)), 6);
td1 = rad2deg(tr1);

% Berechnung von theta 2:
A = -p(3);
B = a(1)-(p(1)*cos(tr1)+p(2)*sin(tr1));
Dn = 2*a(1)*(p(1)*cos(tr1)+p(2)*sin(tr1))+(a(3)^2)-(a(2)^2)-(a(1)^2)-p(3)^2-(p(1)*cos(tr1)+p(2)*sin(tr1))^2;
Dd = 2*a(2);
D = Dn/Dd;
%
phi = real(atan2(B,A));
r = sqrt(A^2+B^2); 
%
rD = [real(sqrt(r^2-D^2));real(-sqrt(r^2-D^2))];
% Insert Geometrp(2) Constriant
% for now use Positive Value
tr2 = round(-phi+atan2(D,rD(1)), 6);
td2 = rad2deg(tr2);

% Berechnung von theta 3:
tr3 = round(atan2(p(3)-a(2)*sin(tr2), p(1)*cos(tr1)+p(2)*sin(tr1)-a(2)*cos(tr2)-a(1))-tr2, 6);
td3 = rad2deg(tr3);

%% Berechnung der Orientierungsgebenden Gelenkstellung d.h. th4, th5 und th6
% A1_E = A1_4 * A4_E
% d.h.
% A1_E * (A1_4)^-1 = A1_4 * A4_E * (A1_4)^-1
% Associativität
% A1_E * (A1_4)^-1 = A4_E * A1_4 * (A1_4)^-1
% A4_E = A1_E * (A1_4)^-1
%
% mit A1_E = |RotationsMatrix(3x3) Verschiebung(3x1)|
%            |      0(1x3)                1         |
%
%   A = [ cos(theta)*cos(beta) -cos(alpha)*sin(theta)+sin(alpha)*sin(beta)*cos(theta)  sin(alpha)*sin(theta)+cos(alpha)*sin(beta)*cos(theta) a*cos(theta);
%         sin(theta)*cos(beta)  cos(alpha)*cos(theta)+sin(alpha)*sin(beta)*sin(theta) -sin(alpha)*cos(theta)+cos(alpha)*sin(beta)*sin(theta) a*sin(theta);
%        -sin(beta)             sin(alpha)*cos(beta)                                   cos(alpha)*cos(beta)                                  d           ;
%         0                     0                                                      0                                                     1           ];
%
% Ermittlung der Manipulator Transformations-Matrix 
[A1_2, A1_3, A1_4] = ArmTrans(td1, td2, td3, 0, 0, 0);

A1_E = Trans3(Pend);
A4_E = A1_E * (A1_4)^-1;

% Berechnung von theta 5
tr5 = real(acos(A4_E(3,1)));
td5 = rad2deg(tr5);

% Berechnung von theta 6
tr6 = asin(A4_E(3,3)/sin(tr5));
td6 = rad2deg(tr6);

% Berechnung von theta 4
C= A4_E(2,1)+ sin(tr5);
tr4 = real(acos(C));;
td4 = rad2deg(tr4);

end

%% Ermittlung der Transformation Matrix entsprechend der DH-Parametern
% <theta> Rotation um die Z/Gelenk-Achse
% <beta> Rotatio um die Y-Achse
% <alpha> Rotation um die X-Achse
% <a> Verschiebung entlang des X/Gelenk-Achse
% <d> Verschiebung entlang der Y-Achse
function TMat = Trans3(p)
% TMat = |RotationsMatrix(3x3) Verschiebung(3x1)|
%        |      0(1x3)                1         |
%
% RMat = R_z * R_y * R_x
% RMat(theta) = |cos(theta) 0 -sin(theta)|   | cos(beta) 0 sin(beta)|   |1 0           0         |
%               |sin(theta) 0  cos(theta)| * |     0      0     0   | * |0 cos(alpha) -sin(alpha)| 
%               |0          0  0         |   |-sin(beta) 0 cos(beta)|   |0 sin(alpha)  cos(alpha)|
% 
% D = |acos(theta)|
%     |asin(theta)|
%     |     d     |

TMat = [ cos(p(6))*cos(p(5)) -cos(p(4))*sin(p(6))+sin(p(4))*sin(p(5))*cos(p(6))  sin(p(4))*sin(p(6))+cos(p(4))*sin(p(5))*cos(p(6)) p(1);
         sin(p(6))*cos(p(5))  cos(p(4))*cos(p(6))+sin(p(4))*sin(p(5))*sin(p(6)) -sin(p(4))*cos(p(6))+cos(p(4))*sin(p(5))*sin(p(6)) p(2);
              -sin(p(5))                   sin(p(4))*cos(p(5))                                   cos(p(4))*cos(p(5))               p(3);
                  0                                 0                                                      0                        1   ];
end
%% Datei: Invkinematik.m
% Beschreibung: Evaluation und Emittlung der Inverse-Kinematik
% Autor: Chukwunonso Bob-Anyeji
% Datum: 27-07-2025@11-52
%
function [td1,td2,td3] = InvKinematik(x,y,z,rx,ry,rz)
clc
% DH Parameter auslesen
[alpha beta a d] =  DhParams();

% Berechnung der Positionsgebender Gelenkwinkel i.e th1, th2 und th3
% 
% TMat = |RotationsMatrix(3x3) Verschiebung(3x1)|
%        |      0(1x3)                1         |
%        
% A1_4 = A1_2 * A2_3 * A3_4
% A1_4 = |cos(th1)*cos(th2+th3) -sin(th2+th3)*cos(th1)  sin(th1) cos(th1)*(a(3)*cos(th2+th3)+a(2)*cos(th2)+a(1))|
%        |sin(th1)*cos(th2+th3) -sin(th2+th3)*sin(th1) -cos(th1) sin(th1)*(a(3)*cos(th2+th3)+a(2)*cos(th2)+a(1))|
%        |    sin(th2+th3)           cos(th2+th3)          0            a(3)*sin(th2+th3)+a(2)*sin(th2)         |
%        |         0                      0                0                           1                        |
%                     
%% Berechnung von theta 1: 
% x = cos(th1)*(a(3)*cos(th2+th3)+a(2)*cos(th2)+a(1))
% y = sin(th1)*(a(3)*cos(th2+th3)+a(2)*cos(th2)+a(1))
% z = a(3)*sin(th2+th3)+a(2)*sin(th2)
tr1 = atan2(y,x);
td1 = rad2deg(tr1);

%% Berechnung von theta 2:
A = -z;
B = a(1)-(x*cos(tr1)+y*sin(tr1));
Dn = 2*a(1)*(x*cos(tr1)+y*sin(tr1))+(a(3)^2)-(a(2)^2)-(a(1)^2)-z^2-(x*cos(tr1)+y*sin(tr1))^2;
Dd = 2*a(2);
D = Dn/Dd
%
phi = atan2(B,A);
r = sqrt(A^2+B^2) 
%
rD = [sqrt(r^2-D^2);-sqrt(r^2-D^2)]
% Insert Geometry Constriant
% for now use Positive Value
tr2 = -phi+atan2(D,rD(1));
td2 = rad2deg(tr2);

%% Berechnung von theta 3:
tr3 = atan2(z-a(2)*sin(tr2), x*cos(tr1)+y*sin(tr1)-a(2)*cos(tr2)-a(1))-tr2;
td3 = rad2deg(tr3);

A

end



























% % Ermittlung der Ziel Position
% [mat04,rot] = fdkinematics(-45.0000, 97.2357, 58.7955, -38.440,'b', '-o')
% 
% % Destination with only Rotation in Y Configurable
% %P4 = [mat04(1,4); mat04(2,4); mat04(3,4); ToRad(rot(1,1)); ToRad(rot(1,2)); ToRad(rot(1,3))];
% P4 = [100; -100; 100; ToRad(0); ToRad(0); ToRad(0)];
% 
% % Configuration of Rotation Matrices
% % Rotation Matrix around the X-Axis
% RxMat = [1,            0,             0;
%          0, cos(P4(4,1)), -sin(P4(4,1));
%          0, sin(P4(4,1)),  cos(P4(4,1))];
% % Rotation Matrix around the Y-Axis
% RyMat = [cos(P4(5,1)), 0, -sin(P4(5,1));
%                     0, 1,             0;
%          sin(P4(5,1)), 0,  cos(P4(5,1))];
% % Rotation Matrix around the Z-Axis
% RzMat = [cos(P4(6,1)), -sin(P4(6,1)), 0;
%          sin(P4(6,1)),  cos(P4(6,1)), 0;
%                     0,             0, 1];
% % Complete Rotation Matrix 
% RMat = RzMat*RyMat*RxMat; 
% 
% % Calculation of theta1
% tr1 = real(atan2(P4(2,1), P4(1,1)));     % Equation (0)
% if(tr1 >= pi/2 || tr1 <= -pi/2)
%     tr1 = tr1-pi
% end
% td1 = ToDegree(tr1);
% 
% % Calculation of Point 3 with by application to Trigonometrical Ratios
% % SOH-CAH-TOA
% adj34 = a4*cos(P4(5,1));
% opp34 = a4*sin(P4(5,1));
% P3 = [P4(1,1) - adj34*cos(tr1);
%       P4(2,1) - adj34*sin(tr1);
%       P4(3,1) + opp34;];
% 
% % Determination of theta2 by Application of the Linear combination of Sin and Cosin waves
% % ie. A*cos(x) + B*sin(x) = C*cos(x + phi) with
% % C = sgn(A).Sqrt(A^2 + B^2) ie. sgn(A) = (A/|A|)
% % phi = arctan(-B/A)
% 
% % First Calculate Total lenght of Adjacent and opposit till point 3 and constants
% adj3 = P3(1,1)/cos(tr1);
% opp3 = P3(3,1);
% A = (adj3 - a1);
% B = opp3;
% 
% % Application of the linear combination
% tr2 = real(acos((abs(A)*(a2^2 - a3^2 + A^2 + opp3^2))/(2*a2*A*sqrt(A^2 + B^2))) - atan(-B/A));
% td2 = ToDegree(tr2);
% 
% % Calculation of theta 3 with by application to Trigonometrical Ratios
% % First Calculate lenght of adjacent and opposite for point 2-3
% adj23 = adj3 - (a2*cos(tr2) + a1);
% opp23 = opp3 - (a2*sin(tr2));
% P2 = [P3(1,1) - adj23*cos(tr1);
%       P3(2,1) - adj23*sin(tr1);
%       P3(3,1) - opp23;];
% 
% % Calculate theta 3
% if(P3(3,1) >= P2(3,1))
%     tr3 = tr2 - real(acos(adj23/a3));
% else
%     tr3 = tr2 + real(acos(adj23/a3));
% end
% td3 = ToDegree(tr3);
% 
% % Calculation of theta 4 with by application to Trigonometrical Ratios
% if(P4(3,1) >= P3(3,1))
%     tr4 = real(acos(adj34/a4)) + tr3 - tr2;
% else
%     tr4 = real(-acos(adj34/a4)) + tr3 - tr2;
% end
% td4 = ToDegree(tr4);
% 
% % Plot Forward Kinematics of Theta Results
% fdkinematics(td1-td1, td2-td2, td3-td3, td4-td4,'m', '-+');
% %fdkinematics(td1-td1/2, td2-td2/2, td3-td3 /2, td4-td4/2,'m', '-+');
% fdkinematics(td1, td2, td3, td4,'r', '-+',true);
% 
% Posture = [td1, td2, td3, td4]
% rot


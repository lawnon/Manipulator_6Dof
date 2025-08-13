function [thr1,thr2,thr3] = PositionCraig(x,y,z,accu)
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
[alpha, beta, a, d] =  DhParams();

% Berechnung von theta 1
thr1 = atan2(y,x);
thr1 = real(thr1);
thr1 = round(thr1,accu);

% Berechnung von theta 2
% Ermittlung von theta2 nach Craig. 1989
% Asin(x) + Bcos(x) = D
% mit A = r*cos(phi)
%     B = r*sin(phi)
% d.h.
%     phi = arctan2(B,A)
%     r = + Sqrt(A^2 + B^2)

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
thr2 = round(thr2,accu);

% Berechnung von theta 3:
thr3 = atan2(z-a(2)*sin(thr2), x*cos(thr1)+y*sin(thr1)-a(2)*cos(thr2)-a(1))-thr2;
thr3 = real(thr3);
thr3 = round(thr3,accu);
end


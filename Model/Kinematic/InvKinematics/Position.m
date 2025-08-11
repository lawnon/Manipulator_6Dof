function [thr1,thr2,thr3] = Position(x,y,z,accu)
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
% Ermittlung von theta2 durch Anwendung der Linear Kombination von Sinus
% und Kosinus wellen
% ie. A*cos(x) + B*sin(x) = C*cos(x + phi) with
% C = sgn(A).Sqrt(A^2 + B^2) ie. sgn(A) = (A/|A|)
% phi = arctan(-B/A)

A = ((2*a(2)*x)/cos(thr1) - 2*a(1)*a(2));
B = (2*a(2)*z);
C = sign(A)*sqrt(A^2+B^2);
D = (a(1)^2)+(a(2)^2)-(a(3)^2)+((x/cos(thr1))^2)+(z^2);
Phi = atan2(-B,A);
% d.h
if (C == 0 || (A==0 && B==0))
    thr2 = 0;
else
    thr2 = acos(D/C)-Phi;  
end

thr2 = real(thr2);
thr2 = round(thr2,accu);

% Berechnung von theta 3:
thr3 = atan2(z-a(2)*sin(thr2), x*cos(thr1)+y*sin(thr1)-a(2)*cos(thr2)-a(1))-thr2;
thr3 = real(thr3);
thr3 = round(thr3,accu);
end


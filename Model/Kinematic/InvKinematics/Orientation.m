function [th4,th5,th6] = Orientation(tmat, accu)
% Ermittlung der Orientierung/Gelenkstellung d.h. th4, th5 und th6
%
% mit TMati_j = |RotationsMatrix(3x3) Verschiebung(3x1)|
%               |      0(1x3)               1(1x1)     |

% Berechnung von theta 4
th4 = atan2(-tmat(1,3),tmat(2,3));
th4 = real(th4);
th4 = round(th4,3);

% Berechnung von theta 4
th5 = atan2(sqrt((tmat(1,3))^2+(tmat(2,3))^2),tmat(3,3));
th5 = real(th5);
th5 = round(th5,3);

% Berechnung von theta 6
th6 = atan2(tmat(3,1),tmat(3,2));
th6 = real(th6);
th6 = round(th6,3);
end


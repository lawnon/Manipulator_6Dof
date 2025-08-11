function [th4,th5,th6] = Orientation(tmat4_6)
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

% Berechnung von theta 4
thr4 = atan2(-tmat4_6(1,3),tmat4_6(2,3));
thr4 = real(thr4);
thr4 = round(thr4,3);
th4  = ToDeg(thr4);

% Berechnung von theta 4
thr5 = atan2(sqrt((tmat4_6(1,3))^2+(tmat4_6(2,3))^2),tmat4_6(3,3));
thr5 = real(thr5);
thr5 = round(thr5,3);
th5  = ToDeg(thr5);

% Berechnung von theta 6
thr6 = atan2(tmat4_6(3,1),tmat4_6(3,2));
thr6 = real(thr6);
thr6 = round(thr6,3);
th6  = ToDeg(thr6);
end


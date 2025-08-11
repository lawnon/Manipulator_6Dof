function [alpha,beta,gamma] = ToAngles(tmat, accu)
% Ermittlung der Orientierung/Gelenkstellung d.h. alpha, beta und gamma
%
% mit TMati_j = |RotationsMatrix(3x3) Verschiebung(3x1)|
%               |      0(1x3)               1(1x1)     |

% Berechnung von beta
beta  = atan2(-tmat(3,1), sqrt((tmat(1,1))^2 + (tmat(2,1)^2)));
beta = real(beta);
beta = round(beta,accu);

% Berechnung von alpha
alpha = atan2( tmat(2,1)/cos(beta), tmat(1,1)/cos(beta));
alpha = real(alpha);
alpha = round(alpha,accu);

% Berechnung von gamma
gamma = atan2( tmat(3,2)/cos(beta), tmat(3,3)/cos(beta));
gamma = real(gamma);
gamma = round(gamma,accu);

% Validierung der Winkelwerte
if (round(beta,3) == round(pi/2,3))
    alpha = 0;
    gamma = atan2(tmat(1,2),tmat(2,2));
    gamma = real(gamma);
    gamma = round(gamma,accu);
end
if (round(beta,3) == -round(pi/2,3))
    alpha = 0;
    gamma = -atan2(tmat(1,2),tmat(2,2));
    gamma = real(gamma);
    gamma = round(gamma,accu);
end
end
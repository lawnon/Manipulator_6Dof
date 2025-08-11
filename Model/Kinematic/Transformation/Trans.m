function [ TMat ] = Trans(tmat, x, y, z,alpha, beta, gamma)
% Ermittlung der Transformation Matrix fur Feste Koordinaten Systeme,... 
% entsprechend der eine gegebene Verschiebung
% <alpha> Rotation um die X-Achse
% <beta> Rotatio um die Y-Achse
% <gammer> Rotation um die Z/Gelenk-Achse
% <a> Verschiebung entlang des X/Gelenk-Achse
% <d> Verschiebung entlang der Y-Achse
% Berechnung erfolgt in Radienwerten
%
% TMat = |RotationsMatrix(3x3) Verschiebung(3x1)|
%        |      0(1x3)                1         |
%
% RMat = R_z(gamma) * R_y(beta) * R_x(alpha)
%
% RMat = |cos(gamma) 0 -sin(gamma)|   | cos(beta) 0 sin(beta)|   |1     0           0     |
%        |sin(gamma) 0  cos(gamma)| * |     0     0     0    | * |0 cos(alpha) -sin(alpha)| 
%        |0          0  0         |   |-sin(beta) 0 cos(beta)|   |0 sin(alpha)  cos(alpha)|
% 
% D = |x|
%     |y|
%     |z|
%

alpha = ToRad(alpha);
beta = ToRad(beta);
gamma = ToRad(gamma);

tver = [ cos(beta)*cos(gamma) -cos(alpha)*sin(gamma)+sin(alpha)*sin(beta)*cos(gamma)  sin(alpha)*sin(gamma)+cos(alpha)*sin(beta)*cos(gamma) x;
         cos(beta)*sin(gamma)  cos(alpha)*cos(gamma)+sin(alpha)*sin(beta)*sin(gamma) -sin(alpha)*cos(gamma)+cos(alpha)*sin(beta)*sin(gamma) y;
        -sin(beta)             sin(alpha)*cos(beta)                                   cos(alpha)*cos(beta)                                  z;
         0                     0                                                      0                                                     1];
TMat = tmat*tver;
end


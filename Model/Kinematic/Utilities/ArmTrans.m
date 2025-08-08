%% Ermittlung der Transformation Matrix des Manipulators 
% entsprechend der DH-Parametern
% <TMat0..i> Entspricht jeweils die Manipulator Transformations-Matrix bis
% zum i-ten Gelenk.
function [TMat] = ArmTrans(theta1, theta2, theta3, theta4, theta5, theta6)
% DH Parameter auslesen
[alpha beta a d] =  DhParams();
%
% Transformations-Matrix Ermitteln pro Gelenk
tmat1_2 = Trans2(deg2rad(alpha(1)), deg2rad(beta(1)), deg2rad(theta1), a(1), d(1));
tmat2_3 = Trans2(deg2rad(alpha(2)), deg2rad(beta(2)), deg2rad(theta2), a(2), d(2));
tmat3_4 = Trans2(deg2rad(alpha(3)), deg2rad(beta(3)), deg2rad(theta3), a(3), d(3));
tmat4_5 = Trans2(deg2rad(alpha(4)), deg2rad(beta(4)), deg2rad(theta4), a(4), d(4));
tmat5_6 = Trans2(deg2rad(alpha(5)), deg2rad(beta(5)), deg2rad(theta5), a(5), d(5));
tmat6_E = Trans2(deg2rad(alpha(6)), deg2rad(beta(6)), deg2rad(theta6), a(6), d(6));
%
% Manipulator Transformations-Matrix Ermitteln
TMat(1) = tmat1_2;
TMat(2) = tmat1_2*tmat2_3;
TMat(3) = tmat1_2*tmat2_3*tmat3_4;
TMat(4) = tmat1_2*tmat2_3*tmat3_4*tmat4_5;
TMat(5) = tmat1_2*tmat2_3*tmat3_4*tmat4_5*tmat5_6;
TMat(6) = tmat1_2*tmat2_3*tmat3_4*tmat4_5*tmat5_6*tmat6_E;
end

%% Ermittlung der Transformation Matrix entsprechend der DH-Parametern
% <alpha> Rotation um die X-Achse
% <beta> Rotatio um die Y-Achse
% <gammer> Rotation um die Z/Gelenk-Achse
% <a> Verschiebung entlang des X/Gelenk-Achse
% <d> Verschiebung entlang der Y-Achse
function TMat = Trans(alpha, gamma, a, d)

% TMat = |RotationsMatrix(3x3) Verschiebung(3x1)|
%        |      0(1x3)                1         |
%
% RMat = R_z(gamma) * R_x(alpha)
%
% RMat = |cos(gamma) 0 -sin(gamma)|   |1     0           0     |
%        |sin(gamma) 0  cos(gamma)| * |0 cos(alpha) -sin(alpha)| 
%        |0          0  0         |   |0 sin(alpha)  cos(alpha)|
% 
% D = |acos(gamma)|
%     |asin(gamma)|
%     |     d     |

TMat = [cos(gamma) -cos(alpha)*sin(gamma) sin(alpha)*sin(gamma)  a*cos(gamma);
        sin(gamma) cos(alpha)*cos(gamma)  -sin(alpha)*cos(gamma) a*sin(gamma);
        0          sin(alpha)             cos(alpha)             d           ;
        0          0                      0                      1           ];
end

%% Ermittlung der Transformation Matrix entsprechend der DH-Parametern
% <alpha> Rotation um die X-Achse
% <beta> Rotatio um die Y-Achse
% <gammer> Rotation um die Z/Gelenk-Achse
% <a> Verschiebung entlang des X/Gelenk-Achse
% <d> Verschiebung entlang der Y-Achse
function TMat = Trans2(alpha, beta, gamma, a, d)

% TMat = |RotationsMatrix(3x3) Verschiebung(3x1)|
%        |      0(1x3)                1         |
%
% RMat = R_z(gamma) * R_y(beta) * R_x(alpha)
%
% RMat = |cos(gamma) 0 -sin(gamma)|   | cos(beta) 0 sin(beta)|   |1     0           0     |
%        |sin(gamma) 0  cos(gamma)| * |     0     0     0    | * |0 cos(alpha) -sin(alpha)| 
%        |0          0  0         |   |-sin(beta) 0 cos(beta)|   |0 sin(alpha)  cos(alpha)|
% 
% D = |acos(gamma)|
%     |asin(gamma)|
%     |     d     |

TMat = [ cos(gamma)*cos(beta) -cos(alpha)*sin(gamma)+sin(alpha)*sin(beta)*cos(gamma)  sin(alpha)*sin(gamma)+cos(alpha)*sin(beta)*cos(gamma) a*cos(gamma);
         sin(gamma)*cos(beta)  cos(alpha)*cos(gamma)+sin(alpha)*sin(beta)*sin(gamma) -sin(alpha)*cos(gamma)+cos(alpha)*sin(beta)*sin(gamma) a*sin(gamma);
        -sin(beta)             sin(alpha)*cos(beta)                                   cos(alpha)*cos(beta)                                  d           ;
         0                     0                                                      0                                                     1           ];
end
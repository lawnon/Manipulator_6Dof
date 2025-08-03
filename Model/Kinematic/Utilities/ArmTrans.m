%% Ermittlung der Transformation Matrix des Manipulators 
% entsprechend der DH-Parametern
% <A1..i> Entspricht jeweils die Manipulator Transformations-Matrix bis
%         zum i-ten Gelenk.
function [A1_2, A1_3, A1_4, A1_5, A1_6, A1_E] = ArmTrans(theta1, theta2, theta3, theta4, theta5, theta6)
% DH Parameter auslesen
[alpha beta a d] =  DhParams();
%
% Transformations-Matrix Ermitteln pro Gelenk
a1_2 = Trans2(deg2rad(theta1), deg2rad(beta(1)), deg2rad(alpha(1)), a(1), d(1));
a2_3 = Trans2(deg2rad(theta2), deg2rad(beta(2)), deg2rad(alpha(2)), a(2), d(2));
a3_4 = Trans2(deg2rad(theta3), deg2rad(beta(3)), deg2rad(alpha(3)), a(3), d(3));
a4_5 = Trans2(deg2rad(theta4), deg2rad(beta(4)), deg2rad(alpha(4)), a(4), d(4));
a5_6 = Trans2(deg2rad(theta5), deg2rad(beta(5)), deg2rad(alpha(5)), a(5), d(5));
a6_E = Trans2(deg2rad(theta6), deg2rad(beta(6)), deg2rad(alpha(6)), a(6), d(6));
%
% Manipulator Transformations-Matrix Ermitteln
A1_2 = a1_2;
A1_3 = a1_2*a2_3;
A1_4 = a1_2*a2_3*a3_4;
A1_5 = a1_2*a2_3*a3_4*a4_5;
A1_6 = a1_2*a2_3*a3_4*a4_5*a5_6;
A1_E = a1_2*a2_3*a3_4*a4_5*a5_6*a6_E;
end

%% Ermittlung der Transformation Matrix entsprechend der DH-Parametern
% <theta> Rotation um die Z/Gelenk-Achse
% <alpha> Rotation um die X-Achse
% <a> Verschiebung entlang des X/Gelenk-Achse
% <d> Verschiebung entlang der Y-Achse
function TMat = Trans(theta, alpha, a, d)
% TMat = |RotationsMatrix(3x3) Verschiebung(3x1)|
%        |      0(1x3)                1         |
%
% RMat = R_z * R_x
% RMat(theta) = |cos(theta) 0 -sin(theta)|   |1 0           0         |
%               |sin(theta) 0  cos(theta)| * |0 cos(alpha) -sin(alpha)| 
%               |0          0  0         |   |0 sin(theta)  cos(theta)|
% 
% D = |acos(theta)|
%     |asin(theta)|
%     |     d     |

TMat = [cos(theta) -cos(alpha)*sin(theta) sin(alpha)*sin(theta)  a*cos(theta);
        sin(theta) cos(alpha)*cos(theta)  -sin(alpha)*cos(theta) a*sin(theta);
        0          sin(alpha)             cos(alpha)             d           ;
        0          0                      0                      1           ];
end

%% Ermittlung der Transformation Matrix entsprechend der DH-Parametern
% <theta> Rotation um die Z/Gelenk-Achse
% <beta> Rotatio um die Y-Achse
% <alpha> Rotation um die X-Achse
% <a> Verschiebung entlang des X/Gelenk-Achse
% <d> Verschiebung entlang der Y-Achse
function TMat = Trans2(theta, beta, alpha, a, d)
% TMat = |RotationsMatrix(3x3) Verschiebung(3x1)|
%        |      0(1x3)                1         |
%
% RMat = R_z * R_y * R_x
% RMat(theta) = |cos(theta) 0 -sin(theta)|   | cos(theta) 0 sin(theta)|   |1 0           0         |
%               |sin(theta) 0  cos(theta)| * |     0      0     0     | * |0 cos(alpha) -sin(alpha)| 
%               |0          0  0         |   |-sin(theta) 0 cos(theta)|   |0 sin(theta)  cos(theta)|
% 
% D = |acos(theta)|
%     |asin(theta)|
%     |     d     |

TMat = [ cos(theta)*cos(beta) -cos(alpha)*sin(theta)+sin(alpha)*sin(beta)*cos(theta)  sin(alpha)*sin(theta)+cos(alpha)*sin(beta)*cos(theta) a*cos(theta);
         sin(theta)*cos(beta)  cos(alpha)*cos(theta)+sin(alpha)*sin(beta)*sin(theta) -sin(alpha)*cos(theta)+cos(alpha)*sin(beta)*sin(theta) a*sin(theta);
        -sin(beta)             sin(alpha)*cos(beta)                                   cos(alpha)*cos(beta)                                  d           ;
         0                     0                                                      0                                                     1           ];
end
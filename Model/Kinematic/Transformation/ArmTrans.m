%% Ermittlung der Transformation Matrix des Manipulators 
% entsprechend der DH-Parametern
% <TMat0..i> Entspricht jeweils die Manipulator Transformations-Matrix bis
% zum i-ten Gelenk.
function [TMat0_1, TMat0_2, TMat0_3, TMat0_4, TMat0_5, TMat0_6] =...
    ArmTrans(theta1, theta2, theta3, theta4, theta5, theta6)
% DH Parameter auslesen
[alpha, beta, a, d] =  DhParams();
%
% Transformations-Matrix Ermitteln pro Gelenk
tmat1_2 = TransFK(alpha(1), beta(1), theta1, a(1), d(1));
tmat2_3 = TransFK(alpha(2), beta(2), theta2, a(2), d(2));
tmat3_4 = TransFK(alpha(3), beta(3), theta3, a(3), d(3));
tmat4_5 = TransFK(alpha(4), beta(4), theta4, a(4), d(4));
tmat5_6 = TransFK(alpha(5), beta(5), theta5, a(5), d(5));
tmat6_E = TransFK(alpha(6), beta(6), theta6, a(6), d(6));
%
% Manipulator Transformations-Matrix Ermitteln
TMat0_1 = tmat1_2;
TMat0_2 = tmat1_2*tmat2_3;
TMat0_3 = tmat1_2*tmat2_3*tmat3_4;
TMat0_4 = tmat1_2*tmat2_3*tmat3_4*tmat4_5;
TMat0_5 = tmat1_2*tmat2_3*tmat3_4*tmat4_5*tmat5_6;
TMat0_6 = tmat1_2*tmat2_3*tmat3_4*tmat4_5*tmat5_6*tmat6_E;
end
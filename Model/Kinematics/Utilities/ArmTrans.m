function TMat = ArmTrans(joint, theta1, theta2, theta3, theta4, theta5, theta6)
arguments
    joint  = 0
    theta1 = 0
    theta2 = 0
    theta3 = 0
    theta4 = 0
    theta5 = 0
    theta6 = 0
end

% Denavite Hartenberg Parameter
% ======================================================================
% alpha01 =  000; a01 = 000.000; d01 = 0.00;
% alpha_i,i+1 = Beschreibt die Rotation (Verdrehung) entlang der X-Acshe
%               zwischen Achse i und i+1 z.B. Achse 1 und 2
% a_i,i+1 = Beschreibt die Translation (Versatzt) Entlang der X-Acshe
%           zwischen Achse i und i+1 z.B. Achse 1 und 2
% d_i,i+1 = Beschreibt die Translation (Versatzt) Entlang der Y-Achse
%           zwischen Achse i und i+1 z.B. Achse 1 und 2

%alpha12 =  090; a12 = 000.000; d12 = 0.00;
alpha12 =  000; a12 = 000.000; d12 = 0.00;
%alpha23 =  090; a23 = 258.300; d23 = 0.00;
alpha23 =  000; a23 = 258.300; d23 = 0.00;
%alpha34 = -090; a34 = 000.000; d34 = 0.00;
alpha34 =  000; a34 = 141.418; d34 = 0.00;
alpha43 = -090; a34 = 000.000; d40 = 0.00;
%alpha45 =  090; a45 = 281.516; d45 = 0.00;
alpha45 =  -090; a45 = 140.100; d45 = 0.00;
%alpha56 = -090; a56 = 000.000; d56 = 0.00;
alpha56 =  000; a56 = 000.000; d56 = 0.00;
alpha6E =  000; a6E = 074.710; d6E = 0.00;

A0_1 = Trans(ToRad(theta1), ToRad(alpha12), a12, d12);
A1_2 = Trans(ToRad(theta2), ToRad(alpha23), a23, d23);
A2_3 = Trans(ToRad(theta3), ToRad(alpha34), a34, d34);
A3_4 = Trans(ToRad(theta4), ToRad(alpha45), a45, d45);
A4_5 = Trans(ToRad(theta5), ToRad(alpha56), a56, d56);
A5_6 = Trans(ToRad(theta6), ToRad(alpha6E), a6E, d6E);

switch joint
    case 1
        TMat = A0_1;
    case 2
        TMat = A0_1*A1_2;
    case 3
        TMat = A0_1*A1_2*A2_3;
    case 4
        TMat = A0_1*A1_2*A2_3*A3_4;
    case 5
        TMat = A0_1*A1_2*A2_3*A3_4*A4_5;
    case 6
        TMat = A0_1*A1_2*A2_3*A3_4*A4_5*A5_6;
end

end
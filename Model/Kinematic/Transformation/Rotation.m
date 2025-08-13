function [RMat] = Rotation(Alpha,Beta,Gamma)
% Ermittlung der Rotations-Matrix,... 
% <Alpha>  {Yaw}   Rotation um die X-Achse
% <Beta>   {Pitch} Rotatio um die Y-Achse
% <Gammer> {Roll} Rotation um die Z/Gelenk-Achse
% Berechnung erfolgt in Radienwerten
%
% TMat = |RotationsMatrix(3x3) Verschiebung(3x1)|
%        |      0(1x3)                1         |
%
% RMat = R_z(Gamma) * R_y(Beta) * R_x(Alpha)
%
% RMat = |cos(Gamma) 0 -sin(Gamma)|   | cos(Beta) 0 sin(Beta)|   |1     0           0     |
%        |sin(Gamma) 0  cos(Gamma)| * |     0     0     0    | * |0 cos(Alpha) -sin(Alpha)| 
%        |0          0  0         |   |-sin(Beta) 0 cos(Beta)|   |0 sin(Alpha)  cos(Alpha)|
% 

Alpha = ToRad(Alpha);
Beta = ToRad(Beta);
Gamma = ToRad(Gamma);

RMat = [ cos(Beta)*cos(Gamma) -cos(Alpha)*sin(Gamma)+sin(Alpha)*sin(Beta)*cos(Gamma)  sin(Alpha)*sin(Gamma)+cos(Alpha)*sin(Beta)*cos(Gamma);
         cos(Beta)*sin(Gamma)  cos(Alpha)*cos(Gamma)+sin(Alpha)*sin(Beta)*sin(Gamma) -sin(Alpha)*cos(Gamma)+cos(Alpha)*sin(Beta)*sin(Gamma);
        -sin(Beta)             sin(Alpha)*cos(Beta)                                   cos(Alpha)*cos(Beta)                                 ];
end


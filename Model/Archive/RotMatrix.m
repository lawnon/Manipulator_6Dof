function RotY = RotMatrix(TMat, theta)
%% Rotation der Transformations-Matrix <TMat> in der Y Achse um Wert theta
%R_y(theta) = | cos(theta) 0 sin(theta)|
%             |     0      0     0     |
%             |-sin(theta) 0 cos(theta)|
%

r_y = [ cos(theta) 0 sin(theta) 0;
            0      0     0      0;
       -sin(theta) 0 cos(theta) 0;
            0      0     0      1;]
RotY = TMat * r_y

end
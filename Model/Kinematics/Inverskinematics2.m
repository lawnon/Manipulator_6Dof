% Defination, Calculation and Verification of the 
% Inverse Kinematics via the Arm Transformation Matrixes
% All Angles in Gradients
clear;
clc;
%clf

% Denavite Hartenberg Parameter
alpha1 =   90; a1 = 28.691; d1 = 0.00;%9.45;
alpha2 = -180; a2 = 58.000; d2 = 0.00;
alpha3 =  180; a3 = 68.300; d3 = 0.00;
alpha4 =    0; a4 = 66.539; d4 = 0.00;%-9.45;

% Destination with only Rotation in Y Configurable
x4 = 165.0488;
y4 = 0;
z4 = 136.3578;
rx4 = ToRad(0);
ry4 = ToRad(-45);
rz4 = ToRad(0);
P4 = [x4, y4, z4];

% Rotation Matrix around the X-Axis
RxMat = [1,       0,        0;
         0, cos(rx4), -sin(rx4);
         0, sin(rx4),  cos(rx4)];
% Rotation Matrix around the Y-Axis
RyMat = [cos(ry4), 0, -sin(ry4);
               0, 1,        0;
         sin(ry4), 0,  cos(ry4)];
% Rotation Matrix around the Z-Axis
RzMat = [cos(rz4), -sin(rz4), 0;
         sin(rz4),  cos(rz4), 0;
               0,        0, 1];
% Complete Rotation Matrix 
RMat = RzMat*RyMat*RxMat; 

% Calculation of theta1
t1 = atan2(y4, x4);     % Equation (0)

% Calculation of link 3 Position (x3, z3)
x3 = x4 - a4*cos(ry4);   % Equation (1a)
y3 = x3 * tan(t1);
z3 = z4 + a4*sin(ry4);   % Equation (1b)
P3 = [x3, y3, z3];

% Calculation of Theta 3 alt
A = real(acos((abs(x3)*(a3^2 - a2^2 + x3^2 + z3^2))/(x3 * realsqrt(x3^2+ z3^2) * 2 * a3)));
B = real(atan((-z3)/(x3)));
t3_alt = A - B;

% Calculation of (x23, z23)
x2 = x3 - a3*cos(t3_alt);   % Equation (1a)
y2 = x2 * tan(t1);
z2 = z3 + a3*sin(t3_alt);   % Equation (1b)
P2 = [x2, y2, z2];

% Calculation of theta 2
t2_alt = real(acos((x2)/(a2)));

if(t2_alt <= 0)
    t2 = 0;       % Equation (10)
    s2 = 0;
else
    t2 = pi - t2_alt;       % Equation (10)
    s2 = pi - (t2_alt + pi/2);
end

% Calculation of Theta 3
if(t2_alt <= 0)
    t3 = 0;
else
    t3 = pi/2 - t3_alt + s2;    % Equation (11)
end

% Calculation of Theta 4
t4 = -(t3_alt + ry4);

% Transform Theta from Grad to Degree
theta1 = ToDegree(t1);
theta2 = ToDegree(t2);
theta2a = ToDegree(t2_alt);
theta3 = ToDegree(t3);
theta3a = ToDegree(t3_alt);
theta4 = ToDegree(t4);

fdkinematics(theta1, theta2, theta3, theta4,'c');
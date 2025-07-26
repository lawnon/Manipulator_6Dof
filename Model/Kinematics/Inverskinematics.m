% Defination, Calculation and Verification of the 
% Inverse Kinematics via the Arm Transformation Matrixes
% All Angles in Gradients
clear;
clc;

% Denavite Hartenberg Parameter
alpha1 =   90; a1 = 28.691; d1 = 0.00;%9.45;
alpha2 = -180; a2 = 58.000; d2 = 0.00;
alpha3 =  180; a3 = 68.300; d3 = 0.00;
alpha4 =    0; a4 = 66.539; d4 = 0.00;%-9.45;

% Destination with only Rotation in Y Configurable
x4 = 221;
y4 = 0;
z4 = 0;
rx4 = ToRad(0);
ry4 = ToRad(0);
rz4 = ToRad(0);

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
z3 = z4 + a4*sin(ry4);   % Equation (1b)

% First Summarization of Constants
c1 = (x3^2 + a3^2 - a2^2 - z3^2)/(2*x3);  % Equation (5)

% Second Summarization of Constants
A = ((z3^2)/(x3^2) + 1);    % Equation (7a)
B = ((2*z3*c1)/x3 - 2*z3);     % Equation (7b)
C = (z3^2 - a3^2 + c1^2);   % Equation (7c)

% Calculation of z2 with the P-Q Formular
z2_p = ((-B/(2*A)) + realsqrt((B^2 - 4*A*C)/(4*A^2)));  % Equation (9a)
z2_m = ((-B/(2*A)) - realsqrt((B ^2 - 4*A*C)/(4*A^2)));  % Equation (9b)
% Determination of z2, Equation (9)
if (abs(z2_m) <= abs(z2_p))
    z2 = z2_m ;
else
    z2 = z2_p;
end

% Calculation of Theta 2 and Sigma 2
t2_alt = (asin(z2/a2));   
t2 = pi - t2_alt;       % Equation (10)
s2 = pi - t2_alt - pi/2;

% Calculation of Theta 3
t3_alt = (asin((z3-z2)/a3));
t3 = pi/2 - t3_alt + s2;    % Equation (11)

% Calculation of Theta 4
t4 = -(t3_alt + ry4);

% Transform Theta from Grad to Degree
theta1 = ToDegree(t1);
theta2 = ToDegree(t2);
theta3 = ToDegree(t3);
theta4 = ToDegree(t4);

%Plot Graphs
hold on;

%Plot Posture of Robot
plot3([0,A0_1(1,4)], [0,A0_1(2,4)], [0,A0_1(3,4)], '-o','Color','b','MarkerSize',10, 'MarkerFaceColor','b', 'LineWidth', 5);
plot3([A0_1(1,4),A0_2(1,4)], [A0_1(2,4),A0_2(2,4)], [A0_1(3,4),A0_2(3,4)], '-o','Color','b','MarkerSize',10, 'MarkerFaceColor','b', 'LineWidth', 5);
plot3([A0_2(1,4),A0_3(1,4)], [A0_2(2,4),A0_3(2,4)], [A0_2(3,4),A0_3(3,4)], '-o','Color','b','MarkerSize',10, 'MarkerFaceColor','b', 'LineWidth', 5);
plot3([A0_3(1,4),A0_4(1,4)], [A0_3(2,4),A0_4(2,4)], [A0_3(3,4),A0_4(3,4)], '-o','Color','b','MarkerSize',10, 'MarkerFaceColor','b', 'LineWidth', 5);

xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
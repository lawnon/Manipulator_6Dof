% Defination and Calculation and Verification of the 
% Forward Kinematics via the Arm Transformation Matrixes
clear;
clc;
clf;

%  A0_400 = [(cos(theta1)*cos(cos(theta2 + theta3) + theta4)) (-cos(theta1)*cos(cos(theta2 + theta3) + theta4)) (sin(theta1))  (cos(theta1)*a4*(cos(theta2 + theta3)*cos(theta2)-cos(theta2 + theta3)*sin(theta4)) + sin(theta1)*d1 + cos(theta1)*(a3*cos(theta2+theta3)+a2*cos(theta2)+a1));
%            (sin(theta1)*cos(cos(theta2 + theta3) + theta4)) (-sin(theta1)*cos(cos(theta2 + theta3) + theta4)) (-cos(theta1)) (sin(theta1)*a4*(cos(theta2 + theta3)*cos(theta2)-cos(theta2 + theta3)*sin(theta4)) - cos(theta1)*d1 + sin(theta1)*(a3*cos(theta2+theta3)+a2*sin(theta2)+a1));
%            (sin(sin(theta2 + theta3) + theta4))             (cos(cos(theta2 + theta3) + theta4))              (0)            (a4*(sin(theta2 + theta3)*cos(theta2)+cos(theta2 + theta3)*sin(theta4)) + a3*sin(theta2+theta3) + a2*sin(theta2)+d1)
%            (0)                                              (0)                                               (0)            (1)                                                                                                                                                          ];

t1 = 0; 
t2 = 0; 
t3 = 0; 
t4 = 0;
t5 = 0;
t6 = 0;
vecTheta1 = zeros(5, 181);

for i = 1:181
    t1 = i - 91;
    A0_6 = ArmTrans(6, t1, t2, t3, t4, t5, t6);
    vecTheta1(1,i) = t1; 
    vecTheta1(2,i) = A0_6(1,4); 
    vecTheta1(3,i) = A0_6(2,4);
    vecTheta1(4,i) = A0_6(3,4);
end

t1 =  45; 
t2 =  45; 
t3 =  0; 
t4 =  0;
t5 =  0;
t6 =  0;

A0_1 = ArmTrans(1, t1)
A0_2 = ArmTrans(2, t1, t2)
A0_3 = ArmTrans(3, t1, t2, t3)
A0_4 = ArmTrans(4, t1, t2, t3, t4)
A0_5 = ArmTrans(5, t1, t2, t3, t4, t5)
A0_6 = ArmTrans(6, t1, t2, t3, t4, t5, t6)

%Plot Graphs
hold on;

%Plot Posture of Robot
plot3([0,A0_1(1,4)]        , [0,A0_1(2,4)]        , [0,A0_1(3,4)]        , '-o','Color','r','MarkerSize',10, 'MarkerFaceColor','b', 'LineWidth', 5);
plot3([A0_1(1,4),A0_2(1,4)], [A0_1(2,4),A0_2(2,4)], [A0_1(3,4),A0_2(3,4)], '-o','Color','g','MarkerSize',10, 'MarkerFaceColor','b', 'LineWidth', 5);
plot3([A0_2(1,4),A0_3(1,4)], [A0_2(2,4),A0_3(2,4)], [A0_2(3,4),A0_3(3,4)], '-o','Color','b','MarkerSize',10, 'MarkerFaceColor','b', 'LineWidth', 5);
plot3([A0_3(1,4),A0_4(1,4)], [A0_3(2,4),A0_4(2,4)], [A0_3(3,4),A0_4(3,4)], '-o','Color','y','MarkerSize',10, 'MarkerFaceColor','b', 'LineWidth', 5);
plot3([A0_4(1,4),A0_5(1,4)], [A0_4(2,4),A0_5(2,4)], [A0_4(3,4),A0_5(3,4)], '-o','Color','m','MarkerSize',10, 'MarkerFaceColor','b', 'LineWidth', 5);
plot3([A0_5(1,4),A0_6(1,4)], [A0_5(2,4),A0_6(2,4)], [A0_5(3,4),A0_6(3,4)], '-o','Color','c','MarkerSize',10, 'MarkerFaceColor','b', 'LineWidth', 5);


view(45,45);
view(0,0);
xlabel('X');
ylabel('Y');
zlabel('Z');
axis([-250 1000 -1000 1000 -250 1000]);
grid on;

%Plot Endeffector Part
plot3(vecTheta1(2,:), vecTheta1(3,:), vecTheta1(4,:), 'o','Color','b','MarkerSize',2, 'MarkerFaceColor','r');
%plot3(vecTheta1(2,:), vecTheta1(3,:), vecTheta1(5,:));

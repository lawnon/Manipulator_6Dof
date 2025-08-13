%% Defination and Calculation and Verification of the 
% Forward Kinematics via the Arm Transformation Matrixes
clear
clc
clf

% Get Arm Transformation Matrix
[A1_2, A1_3, A1_4, A1_5, A1_6, A1_E] = ArmTrans(30, 100, -60, -90, -90, 0);

%% Plot Graphs
%[1] Plot Posture of Robot
subplot(2,2,1);
%
plot3([0,A1_2(1,4)]        , [0,A1_2(2,4)]        , [0,A1_2(3,4)]        , '-o','Color','r','MarkerSize',10, 'MarkerFaceColor','k', 'LineWidth', 5);
plot3([A1_2(1,4),A1_3(1,4)], [A1_2(2,4),A1_3(2,4)], [A1_2(3,4),A1_3(3,4)], '-o','Color','g','MarkerSize',10, 'MarkerFaceColor','k', 'LineWidth', 5);
plot3([A1_3(1,4),A1_4(1,4)], [A1_3(2,4),A1_4(2,4)], [A1_3(3,4),A1_4(3,4)], '-o','Color','b','MarkerSize',10, 'MarkerFaceColor','k', 'LineWidth', 5);
plot3([A1_4(1,4),A1_5(1,4)], [A1_4(2,4),A1_5(2,4)], [A1_4(3,4),A1_5(3,4)], '-o','Color','c','MarkerSize',10, 'MarkerFaceColor','k', 'LineWidth', 5);
plot3([A1_5(1,4),A1_6(1,4)], [A1_5(2,4),A1_6(2,4)], [A1_5(3,4),A1_6(3,4)], '-o','Color','m','MarkerSize',10, 'MarkerFaceColor','k', 'LineWidth', 5);
plot3([A1_6(1,4),A1_E(1,4)], [A1_6(2,4),A1_E(2,4)], [A1_6(3,4),A1_E(3,4)], '-o','Color','y','MarkerSize',10, 'MarkerFaceColor','k', 'LineWidth', 5);
%
view(0, 0);
xlabel('X');
ylabel('Y');
zlabel('Z');
axis([-250 700 -700 700 -250 600]);
grid on;
%
%Plot Endeffector Part
plot3(vecTheta1(2,:), vecTheta1(3,:), vecTheta1(4,:), 'o','Color','b','MarkerSize',2, 'MarkerFaceColor','r');
%plot3(vecTheta1(2,:), vecTheta1(3,:), vecTheta1(5,:));


%[2] Plot Posture of Robot
subplot(2,2,2);
hold on;
%
plot3([0,A1_2(1,4)]        , [0,A1_2(2,4)]        , [0,A1_2(3,4)]        , '-o','Color','r','MarkerSize',10, 'MarkerFaceColor','k', 'LineWidth', 5);
plot3([A1_2(1,4),A1_3(1,4)], [A1_2(2,4),A1_3(2,4)], [A1_2(3,4),A1_3(3,4)], '-o','Color','g','MarkerSize',10, 'MarkerFaceColor','k', 'LineWidth', 5);
plot3([A1_3(1,4),A1_4(1,4)], [A1_3(2,4),A1_4(2,4)], [A1_3(3,4),A1_4(3,4)], '-o','Color','b','MarkerSize',10, 'MarkerFaceColor','k', 'LineWidth', 5);
plot3([A1_4(1,4),A1_5(1,4)], [A1_4(2,4),A1_5(2,4)], [A1_4(3,4),A1_5(3,4)], '-o','Color','c','MarkerSize',10, 'MarkerFaceColor','k', 'LineWidth', 5);
plot3([A1_5(1,4),A1_6(1,4)], [A1_5(2,4),A1_6(2,4)], [A1_5(3,4),A1_6(3,4)], '-o','Color','m','MarkerSize',10, 'MarkerFaceColor','k', 'LineWidth', 5);
plot3([A1_6(1,4),A1_E(1,4)], [A1_6(2,4),A1_E(2,4)], [A1_6(3,4),A1_E(3,4)], '-o','Color','y','MarkerSize',10, 'MarkerFaceColor','k', 'LineWidth', 5);
%
view(90, 0);
xlabel('X');
ylabel('Y');
zlabel('Z');
axis([-250 700 -700 700 -250 600]);
grid on;
%
%Plot Endeffector Part
plot3(vecTheta1(2,:), vecTheta1(3,:), vecTheta1(4,:), 'o','Color','b','MarkerSize',2, 'MarkerFaceColor','r');
%plot3(vecTheta1(2,:), vecTheta1(3,:), vecTheta1(5,:));

%[3] Plot Posture of Robot
subplot(2,2,[3,4]);
hold on;
%
plot3([0,A1_2(1,4)]        , [0,A1_2(2,4)]        , [0,A1_2(3,4)]        , '-o','Color','r','MarkerSize',10, 'MarkerFaceColor','k', 'LineWidth', 5);
plot3([A1_2(1,4),A1_3(1,4)], [A1_2(2,4),A1_3(2,4)], [A1_2(3,4),A1_3(3,4)], '-o','Color','g','MarkerSize',10, 'MarkerFaceColor','k', 'LineWidth', 5);
plot3([A1_3(1,4),A1_4(1,4)], [A1_3(2,4),A1_4(2,4)], [A1_3(3,4),A1_4(3,4)], '-o','Color','b','MarkerSize',10, 'MarkerFaceColor','k', 'LineWidth', 5);
plot3([A1_4(1,4),A1_5(1,4)], [A1_4(2,4),A1_5(2,4)], [A1_4(3,4),A1_5(3,4)], '-o','Color','c','MarkerSize',10, 'MarkerFaceColor','k', 'LineWidth', 5);
plot3([A1_5(1,4),A1_6(1,4)], [A1_5(2,4),A1_6(2,4)], [A1_5(3,4),A1_6(3,4)], '-o','Color','m','MarkerSize',10, 'MarkerFaceColor','k', 'LineWidth', 5);
plot3([A1_6(1,4),A1_E(1,4)], [A1_6(2,4),A1_E(2,4)], [A1_6(3,4),A1_E(3,4)], '-o','Color','y','MarkerSize',10, 'MarkerFaceColor','k', 'LineWidth', 5);
%
view(45, 45);
xlabel('X');
ylabel('Y');
zlabel('Z');
axis([-250 700 -700 700 -250 700]);
grid on;
%
%Plot Endeffector Part
plot3(vecTheta1(2,:), vecTheta1(3,:), vecTheta1(4,:), 'o','Color','b','MarkerSize',2, 'MarkerFaceColor','r');
%plot3(vecTheta1(2,:), vecTheta1(3,:), vecTheta1(5,:));
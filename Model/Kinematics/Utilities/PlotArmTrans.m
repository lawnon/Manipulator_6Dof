function PlotArmTrans(A0_1, A0_2, A0_3, A0_4, cl,zv,path)
arguments
    A0_1 
    A0_2 
    A0_3 
    A0_4 
    cl 
    zv 
    path = false;
end
%Plot Graphs
hold on;

%Plot Posture of Robot
plot3([0,A0_1(1,4)], [0,A0_1(2,4)], [0,A0_1(3,4)], zv,'Color',cl,'MarkerSize',10, 'MarkerFaceColor',cl, 'LineWidth', 5);
plot3([A0_1(1,4),A0_2(1,4)], [A0_1(2,4),A0_2(2,4)], [A0_1(3,4),A0_2(3,4)], zv,'Color',cl,'MarkerSize',10, 'MarkerFaceColor',cl, 'LineWidth', 5);
plot3([A0_2(1,4),A0_3(1,4)], [A0_2(2,4),A0_3(2,4)], [A0_2(3,4),A0_3(3,4)], zv,'Color',cl,'MarkerSize',10, 'MarkerFaceColor',cl, 'LineWidth', 5);
plot3([A0_3(1,4),A0_4(1,4)], [A0_3(2,4),A0_4(2,4)], [A0_3(3,4),A0_4(3,4)], zv,'Color',cl,'MarkerSize',10, 'MarkerFaceColor',cl, 'LineWidth', 5);

if (path)
origin = [221.5300; 0; 0;];
dest = [A0_4(1,4); A0_4(2,4); A0_4(3,4)];
path = [round(linspace(origin(1),dest(1),10));
        round(linspace(origin(2),dest(2),10));
        round(linspace(origin(3),dest(3),10))];
plot3(path(1,:), path(2,:), path(3,:), 'o','Color','b','MarkerSize',2, 'MarkerFaceColor','r');
end


view(45,45);
xlabel('X');
ylabel('Y');
zlabel('Z');
axis([-250 250 -250 250 -200 200]);
grid on;


end
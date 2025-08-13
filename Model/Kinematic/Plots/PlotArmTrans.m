% Datei: PlotArmTrans.m
% Beschreibung: Grafische Dastellung der Roboter-Position/-Stellung
% Autor: Chukwunonso Bob-Anyeji
% Datum: 27-07-2025@11-52
% =========================================================================
function PlotArmTrans(tmat0_1,tmat0_2,tmat0_3,tmat0_4,tmat0_5,tmat0_6...
    ,cl,zv,rx,ry,sz,debug)
% Beschreibung: Grafische Dastellung der Roboter-Position/-Stellung

start = eye(4,4);
start(3,4) = -182.488;
hold on;

plot3([0,0]                      , [0,0]                      , [0,start(3,4)]             , zv, 'Color', cl, 'MarkerSize', sz, 'MarkerFaceColor', 'k', 'LineWidth', sz/2);
plot3([0,tmat0_1(1,4)]           , [0,tmat0_1(2,4)]           , [0,tmat0_1(3,4)]           , zv, 'Color', cl, 'MarkerSize', sz, 'MarkerFaceColor', 'k', 'LineWidth', sz/2);
plot3([tmat0_1(1,4),tmat0_2(1,4)], [tmat0_1(2,4),tmat0_2(2,4)], [tmat0_1(3,4),tmat0_2(3,4)], zv, 'Color', cl, 'MarkerSize', sz, 'MarkerFaceColor', 'k', 'LineWidth', sz/2);
plot3([tmat0_2(1,4),tmat0_3(1,4)], [tmat0_2(2,4),tmat0_3(2,4)], [tmat0_2(3,4),tmat0_3(3,4)], zv, 'Color', cl, 'MarkerSize', sz, 'MarkerFaceColor', 'k', 'LineWidth', sz/2);
plot3([tmat0_3(1,4),tmat0_4(1,4)], [tmat0_3(2,4),tmat0_4(2,4)], [tmat0_3(3,4),tmat0_4(3,4)], zv, 'Color', cl, 'MarkerSize', sz, 'MarkerFaceColor', 'k', 'LineWidth', sz/2);
plot3([tmat0_4(1,4),tmat0_5(1,4)], [tmat0_4(2,4),tmat0_5(2,4)], [tmat0_4(3,4),tmat0_5(3,4)], zv, 'Color', cl, 'MarkerSize', sz, 'MarkerFaceColor', 'k', 'LineWidth', sz/2);
plot3([tmat0_5(1,4),tmat0_6(1,4)], [tmat0_5(2,4),tmat0_6(2,4)], [tmat0_5(3,4),tmat0_6(3,4)], zv, 'Color', cl, 'MarkerSize', sz, 'MarkerFaceColor', 'k', 'LineWidth', sz/2);

view(rx, ry);
xlabel('X');
ylabel('Y');
zlabel('Z');
axis([-850 850 -850 850 -850 850]);
grid on;

if (debug)
    PlotEndPoint(start, 50, '-o');
    PlotEndPoint(tmat0_1, 50, '-o');
    PlotEndPoint(tmat0_2, 50, '-o');
    PlotEndPoint(tmat0_3, 50, '-o');
    PlotEndPoint(tmat0_4, 50, '-o');
    PlotEndPoint(tmat0_5, 50, '-o');
    PlotEndPoint(tmat0_6, 50, '-o');

    origin = [221.5300; 0; 0;];
    dest = [tmat0_6(1,4); tmat0_6(2,4); tmat0_6(3,4)];
    path = [round(linspace(origin(1),dest(1),10));
            round(linspace(origin(2),dest(2),10));
            round(linspace(origin(3),dest(3),10))];
    plot3(path(1,:), path(2,:), path(3,:), 'o','Color','c','MarkerSize',2, 'MarkerFaceColor','r');
end
end
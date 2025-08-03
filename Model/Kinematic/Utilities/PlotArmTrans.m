%% Datei: PlotArmTrans.m
% Beschreibung: Grafische Dastellung der Roboter-Position/-Stellung
% Autor: Chukwunonso Bob-Anyeji
% Datum: 27-07-2025@11-52
%=========================================================================
function PlotArmTrans(a1_2, a1_3, a1_4, a1_5, a1_6, a1_E,cl,zv,rx,ry,drawPath)
hold on;
%
plot3([0,0]                , [0,0]                , [0,-182.488]         , zv, 'Color', cl, 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 5);
plot3([0,a1_2(1,4)]        , [0,a1_2(2,4)]        , [0,a1_2(3,4)]        , zv, 'Color', cl, 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 5);
plot3([a1_2(1,4),a1_3(1,4)], [a1_2(2,4),a1_3(2,4)], [a1_2(3,4),a1_3(3,4)], zv, 'Color', cl, 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 5);
plot3([a1_3(1,4),a1_4(1,4)], [a1_3(2,4),a1_4(2,4)], [a1_3(3,4),a1_4(3,4)], zv, 'Color', cl, 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 5);
plot3([a1_4(1,4),a1_5(1,4)], [a1_4(2,4),a1_5(2,4)], [a1_4(3,4),a1_5(3,4)], zv, 'Color', cl, 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 5);
plot3([a1_5(1,4),a1_6(1,4)], [a1_5(2,4),a1_6(2,4)], [a1_5(3,4),a1_6(3,4)], zv, 'Color', cl, 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 5);
plot3([a1_6(1,4),a1_E(1,4)], [a1_6(2,4),a1_E(2,4)], [a1_6(3,4),a1_E(3,4)], zv, 'Color', cl, 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 5);
%
view(rx, ry);
xlabel('X');
ylabel('Y');
zlabel('Z');
axis([-250 700 -700 700 -250 600]);
grid on;
%
    if (drawPath)
        origin = [221.5300; 0; 0;];
        dest = [a1_E(1,4); a1_E(2,4); a1_E(3,4)];
        path = [round(linspace(origin(1),dest(1),10));
                round(linspace(origin(2),dest(2),10));
                round(linspace(origin(3),dest(3),10))];
        plot3(path(1,:), path(2,:), path(3,:), 'o','Color','b','MarkerSize',2, 'MarkerFaceColor','r');
    end
end
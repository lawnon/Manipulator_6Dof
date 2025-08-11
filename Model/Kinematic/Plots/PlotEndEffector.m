%% Datei: PlotEndPoint.m
% Beschreibung: Grafische Dastellung der Zeil Position
% Autor: Chukwunonso Bob-Anyeji
% Datum: 28-07-2025@11-52
%=========================================================================
function PlotEndEffector(tmat6, tmatE,cl,zv)
hold on;
%
plot3([tmat6(1,4),tmatE(1,4)], [tmat6(2,4),tmatE(2,4)], [tmat6(3,4),tmatE(3,4)], zv, 'Color', cl, 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 5);
end
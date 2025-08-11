%% Datei: PlotEndPoint.m
% Beschreibung: Grafische Dastellung der Zeil Position
% Autor: Chukwunonso Bob-Anyeji
% Datum: 28-07-2025@11-52
%=========================================================================
function PlotEndPoint(tmatS, len, zv)
tmatX = Trans(tmatS, len, 0, 0, 0, 0, 0);
tmatY = Trans(tmatS, 0, len, 0, 0, 0, 0);
tmatZ = Trans(tmatS, 0, 0, len, 0, 0, 0);
hold on;
plot3([tmatS(1,4),tmatX(1,4)], [tmatS(2,4),tmatX(2,4)], [tmatS(3,4),tmatX(3,4)], zv, 'Color', 'r', 'MarkerSize', 2, 'MarkerFaceColor', 'k', 'LineWidth', 1);
plot3([tmatS(1,4),tmatY(1,4)], [tmatS(2,4),tmatY(2,4)], [tmatS(3,4),tmatY(3,4)], zv, 'Color', 'g', 'MarkerSize', 2, 'MarkerFaceColor', 'k', 'LineWidth', 1);
plot3([tmatS(1,4),tmatZ(1,4)], [tmatS(2,4),tmatZ(2,4)], [tmatS(3,4),tmatZ(3,4)], zv, 'Color', 'b', 'MarkerSize', 2, 'MarkerFaceColor', 'k', 'LineWidth', 1);
end
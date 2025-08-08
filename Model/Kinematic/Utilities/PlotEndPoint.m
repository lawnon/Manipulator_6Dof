%% Datei: PlotEndPoint.m
% Beschreibung: Grafische Dastellung der Zeil Position
% Autor: Chukwunonso Bob-Anyeji
% Datum: 28-07-2025@11-52
%=========================================================================
function PlotEndPoint(p,cl,zv)
hold on;
%
plot3(p(1), p(2), p(3), zv, 'Color', cl, 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 5);
end
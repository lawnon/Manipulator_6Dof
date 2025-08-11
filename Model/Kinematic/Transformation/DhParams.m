%% Datei: DhParams.m
% Beschreibung: Definition der Denavit Hartenberg Parameter
%               -------------------------------------------
%               theta_i,i+1 = Beschreibt die vorgabe Rotation (Verdrehung) 
%                             um die Z-Acshe, entlang der Achse i und i+1 
%                             z.B. Achse 1 und 2
%               beta_i,i+1 = Beschreibt die Rotation (Verdrehung) um die 
%                            Y-Achse, entlang der Achse i und i+1 
%                            z.B. Achse 1 und 2
%               alpha_i,i+1 = Beschreibt die Rotation (Verdrehung) um die 
%                             X-Achse, entlang der Achse i und i+1 
%                             z.B. Achse 1 und 2
%               a_i,i+1 = Beschreibt die Translation (Versatzt) Entlang 
%                         der X-Acshe zwischen Achse i und i+1 
%                         z.B. Achse 1 und 2
%               d_i,i+1 = Beschreibt die Translation (Versatzt) Entlang 
%                         der Y-Achse zwischen Achse i und i+1 
%                         z.B. Achse 1 und 2
% Autor: Chukwunonso Bob-Anyeji
% Datum: 27-07-2025@11-52
%=========================================================================
function [alpha beta a d] =  DhParams()
alpha12 = 090; beta12 =  000; a12 = 000.000; d12 = 0.00;
alpha23 = 000; beta23 =  000; a23 = 258.300; d23 = 0.00;
alpha34 = 000; beta34 =  090; a34 = 281.516; d34 = 0.00;
alpha45 = 000; beta45 = -090; a45 = 000.000; d45 = 0.00;
alpha56 = 000; beta56 =  090; a56 = 074.710; d56 = 0.00;
alpha6E = 000; beta6E =  000; a6E = 000.000; d6E = 0.00;
%
alpha = [alpha12; alpha23; alpha34; alpha45; alpha56; alpha6E];
beta = [beta12; beta23; beta34; beta45; beta56; beta6E];
a = [a12; a23; a34; a45; a56; a6E];
d = [d12; d23; d34; d45; d56; d6E];
end
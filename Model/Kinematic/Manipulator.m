%% Matlab Script zur Masterarbeit: 
% Datei: 6DofArm_Kinematik.m
% Beschreibung: Konstruktion, Modellierung und Programmierung der 
%               Inversen Kinematik eines experimentellen Manipulators 
%               mit 6 Freiheitsgraden
% Autor: Chukwunonso Bob-Anyeji
% Datum: 27-07-2025@11-52
% =========================================================================
%

%% Auswertung der Vorwärtskinematik:
clear; clc; clf;

% Denavite und Hartenberg Parametersatz Wählen
dhp = 1;
plot = 1;

% Vorwärtskinematik

%[PosVk,Pos5Vk, TMat06v,RMatv] = VrwKinematik(0,-90,90,45,-90,-45,[0 0.4470 0.7410],'-o',10,plot,dhp);
[PosVk,Pos5Vk, TMat06v,RMatv] = VrwKinematik(0,0,0,0,0,0,[0 0.4470 0.7410],'-o',10,plot,dhp);

%% Auswertung der InverseKinematik:
clear; clc; clf;

% Denavite und Hartenberg Parametersatz Wählen
dhp = 1;
elbow = 1;
plot = 2;

% Rotations(Yaw,Pitch,Roll) Vorgabe
Rot = [70;120;-30];
rmat = Rotation(Rot(1),Rot(2),Rot(3)); 

% Inverse Kinematik 
Pos = [-200;-100;250;Rot(1);Rot(2);Rot(3)];
[td1,td2,td3,td4,td5,td6] = InvKinematik(Pos(1),Pos(2),Pos(3),rmat,plot,dhp,elbow); 
Angles = [td1;td2;td3;td4;td5;td6];

% Validierung durch Vorwärtskinematik
[PosIk,Pos5Ik,TMat06v,RMati] = VrwKinematik(...
    td1,td2,td3,td4,td5,td6,[0.3010 0.7450 0.9330],'-x',10,plot,dhp);

% Positions Difference Herausrechnen und mit loggen
diff = abs(Pos-PosIk);
Log = [Pos PosIk diff Angles];

%% Validierung der InverseKinematik anhand der Vorwärts-Kinemati:
clear; clc; clf;

% Denavite und Hartenberg Parametersatz Wählen
dhp = 1;
elbow = 1;
plot = 2;

% Vorwärts Kinematik
[PosVk,Pos5Vk, TMat06v,RMatv] = VrwKinematik(...
    60,100,-80,45,70,-220,[0 0.4470 0.7410], '-+',10,plot,dhp);

% Inverse Kinematik
[td1,td2,td3,td4,td5,td6] = InvKinematik(...
    PosVk(1),PosVk(2),PosVk(3),RMatv,plot,dhp,elbow);
Angles = [td1;td2;td3;td4;td5;td6];

% Validierung durch Vorwärtskinematik
[PosIk,Pos5Ik,TMat06v,RMati] = VrwKinematik(...
    td1,td2,td3,td4,td5,td6,[0.3010 0.7450 0.9330],'-x',10,plot,dhp);

% Positions Difference Herausrechnen und mit loggen
diff = abs(PosVk-PosIk);
Log = [PosVk PosIk diff Angles];

%% Tabellarische Validierung der InverseKinematik mithilfe von SollPositionen:
clear; clc; clf;

dhp = 1;
elbow = 1;
plot = 0;
itr = 1;
Emat = eye(4,4);

if (plot >0)
    hold on;
    view(45, 45);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis([-650 650 -650 650 -650 650]);
    grid on;
end

for x = [-500:10:500]
    for y = [-500:10:500]
        for z = [-350:10:350]
            for alpha = [-180:10:180]
                for beta = [-90:10:90]
                    for gamma = [-180:10:180]
                        % Rotations(Yaw,Pitch,Roll) Vorgabe
                        Rot = [alpha;beta;gamma];
                        rmat = Rotation(Rot(1),Rot(2),Rot(3));

                        % Inverse Kinematik
                        Pos = [x;y;z;Rot(1);Rot(2);Rot(3)];
                        [td1,td2,td3,td4,td5,td6] = InvKinematik(Pos(1),Pos(2),Pos(3),rmat,plot,dhp,elbow);
                        Angles = [td1;td2;td3;td4;td5;td6];

                        % Validierung durch Vorwärtskinematik
                        [PosIk,Pos5Ik,TMat06v,RMati] = VrwKinematik(...
                            td1,td2,td3,td4,td5,td6,[0.3010 0.7450 0.9330],'-x',10,plot,dhp);

                        % Koordinate Plotten
                        Kmat = Trans(Emat,Pos(1),Pos(2),Pos(3),Pos(4),Pos(5),Pos(6));
                        if (plot >0)
                            %PlotEndPoint(Kmat,50,'-o');
                        end

                        % Positions Difference Herausrechnen und mit loggen
                        diff = abs(Pos-PosIk);
                        Log(itr,1:6) = transpose(Pos);
                        Log(itr+1,1:6) = transpose(PosIk);
                        Log(itr+2,1:6) = transpose(diff);
                        Log(itr+3,1:6) = transpose(Angles);
                        itr = itr + 5;
                    end
                end
            end
        end
    end
end

%% Tabellarische Validierung der InverseKinematik mithilfe von Sollwinkeln:
clear; clc; clf;

dhp = 1;
elbow = 1;
plot = 0;
itr = 1;
itr_Nr = 1;
Emat = eye(4,4);

if (plot >0)
    hold on;
    view(45, 45);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis([-650 650 -650 650 -650 650]);
    grid on;
end

%parpool(2)

for th1 = [-170:20:170]
    for th2 = [-90:10:90]
        for th3 = [-90:10:90]
            for th4 = [-170:20:170]
                for th5 = [-90:10:90]
                    for th6 = [-170:20:170]
                        % Vorwärts Kinematik
                        [PosVk,Pos5Vk, TMat06v,RMatv] = VrwKinematik(...
                            th1,th2,th3,th4,th5,th6,[0 0.4470 0.7410], '-+',10,plot,dhp);

                        % Inverse Kinematik
                        [th1_ik,th2_ik,th3_ik,th4_ik,th5_ik,th6_ik] = InvKinematik(PosVk(1),PosVk(2),PosVk(3),RMatv,plot,dhp,elbow);

                        % Validierung durch Vorwärtskinematik
                        [PosIk,Pos5Ik,TMat06v,RMati] = VrwKinematik(...
                            th1_ik,th2_ik,th3_ik,th4_ik,th5_ik,th6_ik,[0.3010 0.7450 0.9330],'-x',10,plot,dhp);

                        % Koordinate Plotten
                        if (plot>0)
                            Kmat = Trans(Emat,Pos(1),Pos(2),Pos(3),Pos(4),Pos(5),Pos(6));
                            PlotEndPoint(Kmat,50,'-o');
                        end

                        % Positions Difference Herausrechnen und mit loggen
                        diffTheta = abs([th1,th2,th3,th4,th5,th6]-[th1_ik,th2_ik,th3_ik,th4_ik,th5_ik,th6_ik]);
                        diffPos = abs(PosVk-PosIk);

                        Log(itr,1) = itr_Nr;
                        Log(itr+1,1:6) = [th1,th2,th3,th4,th5,th6];
                        Log(itr+2,1:6) = [th1_ik,th2_ik,th3_ik,th4_ik,th5_ik,th6_ik];
                        Log(itr+3,1:6) = transpose(diffTheta);
                        Log(itr+4,1:6) = transpose(PosVk);
                        Log(itr+5,1:6) = transpose(PosIk);
                        Log(itr+6,1:6) = transpose(diffPos);             

                        itr = itr + 7;
                        itr_Nr = itr_Nr + 1;
                    end
                end
            end
        end
    end
end
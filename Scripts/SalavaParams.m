% Parâmetros

% g = 9.81;

Mh = 0.149; %kg
Mr = 0.144; %kg
L = 0.14298; %m
d = 0.0987; %m
i_stall = 1.8; %A
torque_stall = 0.3136; %Nm
w2_noload = 380; %RPM
i_noload = 0.1; %A
r = 100e-3; %m
r_in = r-(8.9e-3); % CORRECTION (mm --> m).
u_nom = 12; %V
Jh = 6.25e-4 / 4;

% Salva parametros
save("..\Parametros\parametros_planta.mat","Mh","Mr","L","d","i_stall","torque_stall","w2_noload","u_nom", ...
    "i_noload","r","r_in","u_nom", "Jh")
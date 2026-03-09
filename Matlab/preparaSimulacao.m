clear all; clc;
load("Parametros\parametros_planta.mat");
load("Parametros\K.mat");

%Ts = 1/5e3;
xp_ic = [0;0];
x_ic = [deg2rad(170); 0;];

% ESC parameters for energy-pumping architecture
esc_amp   = 0.05;          % Perturbation amplitude on k
mod_freq  = 2*pi*15;       % ~94 rad/s perturbation frequency
w_hf      = mod_freq/5;    % ~19 rad/s high-pass cutoff
w_lf      = mod_freq/5;    % ~19 rad/s low-pass cutoff
esc_gain  = 500;           % Start here, adjust as needed
k_init    = 0.3;           % Initial guess for k
%% setup
clear all; close all; clc;
%% mechanical system parameters
J_m = 0.001274; % rotational inertia of the motor and first pulley (kg-m^2)
J_L = 0.01135; % rotational inertia of the second pulley and the load (kg-m^2)
r_p = 4e-2; % radius of the pulleys (m)
K = 300e3; % spring constant (N/m)
%% set up base transfer function
s = tf("s");
%% design inner loop controller
R_1 = r_p; R_2 = r_p; % pulley radii
SEA = (K * R_1 * R_2)/(K * R_1^2 + J_m * s^2); % the plant (in this case the SEA device)
%inner_loop_controller = pidtune(SEA, "PID");
pidTuner(SEA, "PID")
%% design outer loop controller
Z_L = 1/(J_L * s^2); % impedance of the load
%outer_loop_controller = pidtune(Z_L, "PD");
pidTuner(Z_L, "PD");
%% setup
clear all; close all; clc;
%% mechanical system parameters
J_m = 0.001274; % rotational inertia of the motor and first pulley (kg-m^2)
J_L = 0.01135; % rotational inertia of the second pulley and the load (kg-m^2)
r_p = 4e-2; % radius of the pulleys (m)
K = 300e3; % spring constant (N/m)
%% set up base transfer function
s = tf("s");
%% inner loop
R_1 = r_p; R_2 = r_p; % pulley radii
SEA = (K * R_1 * R_2)/(K * R_1^2 + J_m * s^2); % the plant (in this case the SEA device)
inner_loop_controller = pidtune(SEA, "PID");
%% evaluate inner loop controller
closed_inner_loop = feedback(series(inner_loop_controller, SEA), 1);
h = stepplot(closed_inner_loop);
h.showCharacteristic('SettlingTime')
h.showCharacteristic('PeakResponse')
closed_inner_loop_info = stepinfo(closed_inner_loop)
%% outer loop
Z_L = 1/(J_L * s^2); % impedance of the load
outer_loop_controller = pidTuner(Z_L, "PD");
%% evaluate outer loop controller
closed_outer_loop = feedback(series(outer_loop_controller, Z_L), 1);
h = stepplot(closed_outer_loop);
h.showCharacteristic('SettlingTime')
h.showCharacteristic('PeakResponse')
closed_outer_loop_info = stepinfo(closed_outer_loop)
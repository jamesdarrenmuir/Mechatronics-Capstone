%% setup
clear all; close all; clc;
%% system parameters
J_m = 0; % rotational inertia of the motor (kg-m^2)
J_p1 = 0; % rotational inertia of the first pulley (kg-m^2)
J_p2 = 0; % rotational inertia of the second pulley (kg-m^2)
R_1 = 0; % radius of the first pulley (m)
R_2 = 0; % radius of the second pulley (m)
K_1 = 0; % spring constant of the first spring (N/m)
K_2 = 0; % spring constant of the second spring (N/m)
%%
s = tf("s");
Z_L = 0; % impedance of the load (some function of s)
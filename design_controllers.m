%% setup
clear all; close all; clc;
%% mechanical system parameters
%TODO: update these values with values from the CAD model
J_m = 1; % (kg-m^2) moment of inertia of the motor
J_p1 = 1; % (kg-m^2) moment of inertia of the pulley closest to the motor
J_p2 = 1; % (kg-m^2) moment of inertia of the pulley furthest from the motor
J_l = 1; % (kg-m^2) moment of inertia of the load

J_1 = J_m + J_p1;
J_2 = J_p2 + J_l;

r_1 = 1; % (m) radius of the pulley closest to the motor
r_2 = 1; % (m) radius of the pulley furthest from the motor

%TODO: update these values based on the selected springs
K_1 = 1; % (N/m) spring constant of upper spring
K_2 = 1; % (N/m) spring constant of lower spring

K = (K_1*K_2)/(K_1+K_2);

%TODO: update this with values from the real amplifier
K_vi = 0.41; % (A/V) amplifier constant
%TODO: update this value with the value from the selected motor data sheet
K_t = 0.11; % (N-m/A) motor torque constant
%TODO: update this if we use a different device
maximum_output_voltage = 10; % (V) maximum output voltage of myRIO

%TODO: Update these legacy values
J_1 = 0.001274; % rotational inertia of the motor(kg-m^2)
J_2 = 0.01135; % rotational inertia of the second pulley and the load (kg-m^2)
r_p = 4e-2; % radius of the pulleys (m)
K = 300e3; % spring constant (N/m)
%% set up base transfer function
s = tf("s");
%% design controller
R_1 = r_p; R_2 = r_p; % pulley radii
SEA = (K * R_1 * R_2)/(K * R_1^2 + J_m * s^2); % the plant (in this case the SEA device)
inner_loop_plant = K_vi * K_t * SEA; % add the amplifier and torque constants
inner_loop_controller = pidtune(inner_loop_plant, "PIDF");
%pidTuner(inner_loop_transfer_function, "PIDF")
%% design outer loop controller
Z_L = 1/(J_L * s^2); % impedance of the load
outer_loop_controller = pidtune(Z_L, "PDF");
%pidTuner(Z_L, "PD");
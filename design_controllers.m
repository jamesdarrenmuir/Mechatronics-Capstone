%% setup
clear all; close all; clc;
%% mechanical system parameters
%TODO: update these values with values from the CAD model
J_m = 3.8e-4; % (kg-m^2) moment of inertia of the motor
J_p1 = 3.8e-4; % (kg-m^2) moment of inertia of the pulley closest to the motor
J_p2 = 3.8e-4; % (kg-m^2) moment of inertia of the pulley furthest from the motor
J_l = 3.8e-4; % (kg-m^2) moment of inertia of the load

J_1 = J_m + J_p1;
J_2 = J_p2 + J_l;

R_1 = .06; % (m) radius of the pulley closest to the motor
R_2 = .06; % (m) radius of the pulley furthest from the motor

%TODO: update this to the 16:1 speed reduction
R_g = 16; % gearbox ratio

%TODO: update these values based on the selected springs
K_1 = 30; % (N/m) spring constant of upper spring
K_2 = 30; % (N/m) spring constant of lower spring

K = (K_1*K_2)/(K_1+K_2);

%TODO: update this with values from the real amplifier
K_vi = 0.41; % (A/V) amplifier constant
%TODO: update this value with the value from the selected motor data sheet
K_t = 0.11; % (N-m/A) motor torque constant
%TODO: update this if we use a different device
maximum_output_voltage = 10; % (V) maximum output voltage of myRIO
%% set up base transfer function
s = tf("s");
%% design controller
plant = K_vi * K_t * (R_g^2*(J_2*s^2+R_2^2*K))/(R_1^2*J_2*K*s+J_1*s*R_g^2*(J_2*s^2+R_2^2*K));
controller = pidtune(plant, "PIDF", 100);
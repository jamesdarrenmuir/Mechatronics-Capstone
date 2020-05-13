%% setup
clear all; close all; clc;
%% find maximum output voltage
maximum_continuous_output_torque = 0.5; % (N-m) maximum target output torque
Kvi = 0.41; % (A/V) amplifier constant (amplifier from ME 477 lab)
Kt = 0.0214; % (N-m/A) motor torque constant (for our Maxon motor)
Rg = 16; % gearbox ratio, 16:1 speed reduction
% T = V * Kvi * Kt * Rg
Vmax = maximum_continuous_output_torque / (Kvi * Kt * Rg) % (V)
%% find linear spring constants
r = 0.02; % 2 cm pulley radius (m)

% Hebi R8-3
hebi_max_continuous_torque = 3; % (N-m)
hebi_K_rotational = 50; % (N-m/rad)

target_K_rotational = hebi_K_rotational * maximum_continuous_output_torque / hebi_max_continuous_torque;

target_K = target_K_rotational / r^2; % (N/m)
K1 = target_K/2 % (N/m)
K2 = K1 % (N/m)
% K1 = 796;
% K2 = K1;
%% find load mass
target_resonant_frequency = 2*2*pi; % 2 Hz to (rad/s)
% w = sqrt(J/Krot)
J = target_resonant_frequency^2/target_K
load_arm_length = 0.08; % 8 cm (m)
% J = m*d^2
m = J/load_arm_length^2
%% Hebi maximum continuous torques vs miniumum stiffnesses
% hebi_torques = [3 9 16];
% hebi_stiffnesses = [50 70 140];
% plot(hebi_torques, hebi_stiffnesses)
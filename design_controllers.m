%% setup
clear all; close all; clc;
%% mechanical system parameters
%TODO: update these values with values from the CAD model
Jm = 3.8e-4; % (kg-m^2) moment of inertia of the motor
Jp1 = 3.8e-4; % (kg-m^2) moment of inertia of the pulley closest to the motor
Jp2 = 3.8e-4; % (kg-m^2) moment of inertia of the pulley furthest from the motor
Jl = 3.8e-4; % (kg-m^2) moment of inertia of the load

R1 = .06; % (m) radius of the pulley closest to the motor
R2 = .06; % (m) radius of the pulley furthest from the motor

%TODO: update this to the 16:1 speed reduction
Rg = 16; % gearbox ratio

%TODO: update these values based on the selected springs
K1 = 30; % (N/m) spring constant of upper spring
K2 = 30; % (N/m) spring constant of lower spring

%TODO: update this with values from the real amplifier
Kvi = 0.41; % (A/V) amplifier constant
%TODO: update this value with the value from the selected motor data sheet
Kt = 0.11; % (N-m/A) motor torque constant
%TODO: update this if we use a different device
maximum_output_voltage = 10; % (V) maximum output voltage of myRIO
%% plant transfer function
s = tf("s");
K = K1 + K2;
J4 = Jp2 + Jl;
Z1 = 1/(Jm*s);
Z2 =1/(Jp2*s);
Z3 = s/K;
Z4 = 1/(J4*s);
Zp2 = 1/(Jp2*s);
Zl = 1/(Jl*s);
plant = Kvi * Kt * (R1*R2*Z1*Z2*Zp2)/((Z1*R2^2*Z2+(Z1+Rg^2*Z2)*(Z3+R2^2*Z4))*(Zp2+Zl));
%% design controller
opt = pidtuneOptions("PhaseMargin", 75); % Default: 60 deg
controller = pidtune(plant, "PIDF") %, 80, opt);
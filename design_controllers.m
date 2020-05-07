%% setup
clear all; close all; clc;
%% mechanical system parameters
%TODO: update these values with values from the CAD model
Jm = 2.14e-6; % (kg-m^2) moment of inertia of the motor
Jg = 1.31e-7; % (kg-m^2) moment of inertia of the gearbox (as seen by the motor)
Jp1 = 3.297e-6; % (kg-m^2) moment of inertia of the pulley closest to the motor
Jp2 = Jp1; % (kg-m^2) moment of inertia of the pulley furthest from the motor
JL = 1.072e-4; % (kg-m^2) moment of inertia of the load

R1 = 2e-2; % (m) radius of the pulley closest to the motor
R2 = R1; % (m) radius of the pulley furthest from the motor

Rg = 16; % gearbox ratio, 16:1 speed reduction

%TODO: update these values based on the selected springs
K1 = 18913; % (N/m) spring constant of upper spring
K2 = K1; % (N/m) spring constant of lower spring

%TODO: update this with values from the real amplifier
Kvi = 0.41; % (A/V) amplifier constant
%TODO: update this value with the value from the selected motor data sheet
Kt = 0.0214; % (N-m/A) motor torque constant
%TODO: update this if we use a different device
maximum_output_voltage = 10; % (V) maximum output voltage of myRIO
%% single loop controller
s = tf("s");
K = K1 + K2;
J1 = Jm + Jg;
J2 = Jp2 + JL;
plant = Kvi * Kt * (K*R1*R2^2*Rg)/(R2*s*(K*R1^2*J2*s+Jp1*s*(J2*s^2+K*R2^2)+J1*Rg^2*s*(J2*s^2+K*R2^2)));
plant = minreal(plant);

opt = pidtuneOptions("PhaseMargin", 70); % Default: 60 deg
single_loop_controller = pidtune(plant, "PDF", 5, opt);
%% double loop controllers
%% inner loop controller
KKSEA = Kvi * Kt * K*R1*R2*Rg/(K*R1^2+(Jp1+J1*Rg^2)*s^2);
KKSEA = minreal(KKSEA);
opt = pidtuneOptions("PhaseMargin", 75); % Default: 60 deg
inner_loop_controller = pidtune(KKSEA, "PIDF", .01, opt);
%% outer loop controller
ZL = 1/(J2*s^2);
opt = pidtuneOptions("PhaseMargin", 60); % Default: 60 deg
outer_loop_controller = pidtune(ZL, "PDF", 30, opt);
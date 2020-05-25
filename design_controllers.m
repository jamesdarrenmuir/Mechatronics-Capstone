%% setup
clear all; close all; clc;
%% mechanical system parameters
% these values are determined by the hardware we pick
%TODO: update these values based on the final hardware

% motor
Kt = 0.0214; % (N-m/A) motor torque constant
MCOT = 0.5; % (N-m) maximum continuous output torque
Jm = 2.14e-6; % (kg-m^2) moment of inertia of the motor

% gearbox
Rg = 16; % gearbox ratio, 16:1 speed reduction
Jg = 1.31e-7; % (kg-m^2) moment of inertia of the gearbox (as seen by the motor)

% pulleys
% pulley 1
R1 = 2e-2; % (m) radius of the pulley closest to the motor
Jp1 = 3.297e-6; % (kg-m^2) moment of inertia of the pulley closest to the motor
% pulley 2
R2 = R1; % (m) radius of the pulley furthest from the motor
Jp2 = Jp1; % (kg-m^2) moment of inertia of the pulley furthest from the motor

% load
JL = .007; % (kg-m^2) moment of inertia of the load

% springs
K1 = 1e4; % (N/m) spring constant of upper spring
K2 = K1; % (N/m) spring constant of lower spring

% amplifier
Kvi = 0.41; % (A/V) amplifier constant (from ME 477 lab)

% myRIO
maximum_output_voltage = 10; % (V) maximum output voltage of myRIO
%% transfer function set up
s = tf("s");
K = K1 + K2;
J1 = Jm + Jg;
J2 = Jp2 + JL;
%% single loop controller

% set up plant
single_loop_plant = Kvi * Kt * K*R1*R2*Rg/(J2*(Rg^2*J1+Jp1)*s^4+K*(Rg^2*R2^2*J1+Jp1*R2^2+J2*R1^2)*s^2);
single_loop_plant = minreal(single_loop_plant);

% design controller
opt = pidtuneOptions("PhaseMargin", 60); % Default: 60 deg
single_loop_controller = pidtune(single_loop_plant, "PDF", 1, opt);

% evaluate single loop controller
% output response
figure('NumberTitle', 'off', 'Name', 'Single Loop Output');
closed_loop = feedback(series(single_loop_controller, single_loop_plant), 1);
opt = stepDataOptions('StepAmplitude', pi/4); % 45 deg rotation step
h = stepplot(closed_loop, opt);
xlim([0 100])
title("Output vs Time")
ylabel("Position (rad)")
h.showCharacteristic('SettlingTime')
h.showCharacteristic('PeakResponse')
% single_loop_info = stepinfo(closed_loop)
% single_loop_bandwidth = bandwidth(closed_loop)

% effort response
figure('NumberTitle', 'off', 'Name', 'Single Loop Effort');
controller_effort_transfer_function = feedback(tf(single_loop_controller), single_loop_plant);
step(controller_effort_transfer_function, opt)
title("Effort vs Time")
ylabel("Voltage (V)")
%% double loop controllers
% the inner loop is nested inside the outer loop
%% inner loop controller
% set up plant
inner_loop_plant = Kvi * Kt * K*R1*R2*Rg/(K*R1^2+(Jp1+J1*Rg^2)*s^2);
inner_loop_plant = minreal(inner_loop_plant);

% design controller
% 10% OS -> zeta = 0.6
zeta = .6; % Garbini's recommendation
% zeta = .33;
Ts = 0.03; % (s) Garbini's recommendation
% Ts = 0.08;
wn = 4 / (Ts * zeta); % (rad/s)

z = 80; % Garbini's recommendation
% z = 25;

tp = -zeta*wn + wn*sqrt(1-zeta^2)*1j; % target pole
ps = pole(inner_loop_plant); % open loop poles
ang = angle(tp + z) - angle(tp - ps(1)) ...
    - angle(tp - ps(2)) - (2*-1+1)*pi; % angle from tp to p
p = imag(tp)/atan(ang) - real(tp); % controller pole
[~,Kp] = zero(inner_loop_plant); % plant gain
Kc = abs(tp-ps(1))*abs(tp-ps(2))*abs(tp+p)/abs(tp+z)/Kp; % controller gain
inner_loop_controller = Kc*(s+z)/(s+p);

% visualize controller design
figure('NumberTitle', 'off', 'Name', 'Inner Loop Controller Target Vs Result');
open_loop_tf = series(inner_loop_controller, inner_loop_plant);
rlocus(open_loop_tf)
hold on
xline(-zeta*wn, 'k:')
sgrid(zeta,0)
h2 = plot([tp conj(tp)], 'kx', 'MarkerSize',12);
r = rlocus(open_loop_tf, 1);
h1 = plot(r,'gs',    'MarkerEdgeColor','k',...
                    'MarkerFaceColor','g',...
                    'MarkerSize',9);
legend([h1 h2] , ["closed loop poles" "target poles"])

% evaluate inner loop controller
% output response
figure('NumberTitle', 'off', 'Name', 'Inner Loop Output');
closed_loop = feedback(series(inner_loop_controller, inner_loop_plant), 1);
maximum_motor_output_torque = maximum_output_voltage * Kvi * Kt;
opt = stepDataOptions('StepAmplitude', MCOT);
h = stepplot(closed_loop, opt);
hold on
y = yline(MCOT);
ylim([-1.1*MCOT 1.1*MCOT])
legend(y, 'Target')
title("Output vs Time")
ylabel("Torque (N-m)")
h.showCharacteristic('SettlingTime')
h.showCharacteristic('PeakResponse')
% inner_loop_info = stepinfo(closed_loop)
% inner_loop_bandwidth = bandwidth(closed_loop)

% effort response
figure('NumberTitle', 'off', 'Name', 'Inner Loop Effort');
controller_effort_transfer_function = feedback(tf(inner_loop_controller), inner_loop_plant);
step(controller_effort_transfer_function, opt)
title("Effort vs Time")
ylabel("Voltage (V)")
%% outer loop controller
% set up plant
Z2 = 1/(J2*s^2);
outer_loop_plant = series(feedback(series(inner_loop_controller, inner_loop_plant), 1), Z2);

% design controller
% output torque < 0.5 N-m
opt = pidtuneOptions("PhaseMargin", 60); % Default: 60 deg
outer_loop_controller = pidtune(outer_loop_plant, "PIDF", 4, opt);

% evaluate outer loop controller
% output response
figure('NumberTitle', 'off', 'Name', 'Outer Loop Output');
closed_loop = feedback(series(outer_loop_controller, outer_loop_plant), 1);
opt = stepDataOptions('StepAmplitude', pi/4); % 45 deg rotation step
h = stepplot(closed_loop, opt);
title("Output vs Time")
ylabel("Position (rad)")
h.showCharacteristic('SettlingTime')
h.showCharacteristic('PeakResponse')
% outer_loop_info = stepinfo(closed_loop)
% outer_loop_bandwidth = bandwidth(closed_loop)

% effort response
figure('NumberTitle', 'off', 'Name', 'Outer Loop Effort');
controller_effort_transfer_function = feedback(tf(outer_loop_controller), outer_loop_plant);
step(controller_effort_transfer_function, opt)
title("Effort vs Time")
ylabel("Torque (N-m)")
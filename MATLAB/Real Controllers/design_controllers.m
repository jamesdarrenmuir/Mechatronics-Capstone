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
% as seen by the motor
Jg = 1.31e-7; % (kg-m^2) moment of inertia of the gearbox

% shaft couplers
Js = 8.6e-7; % (kg-m^2) moment of inertia of the shaft couplers

% encoder
% 500 windows * 4 ticks per window for a quadrature encoder
BDI_per_rev = 500 * 4; % (BDI/rev) 

% pulleys
% pulley 1 (closest to the motor)
R1 = 2e-2; % (m) radius of the pulley closest to the motor
Jp1 = 2.846e-6; % (kg-m^2) moment of inertia of the pulley 
% pulley 2 (furthest from the motor)
R2 = R1; % (m) radius of the pulley furthest from the motor
Jp2 = Jp1; % (kg-m^2) moment of inertia of the pulley

% load
JL = .00101; % (kg-m^2) moment of inertia of the load

% springs
K1 = 578; % (N/m) spring constant of upper spring
K2 = K1; % (N/m) spring constant of lower spring

% amplifier
Kvi = 0.41; % (A/V) amplifier constant (from ME 477 lab)

% myRIO
maximum_output_voltage = 10; % (V) maximum output voltage of myRIO
%% transfer function set up
s = tf("s");
K = K1 + K2;
J1 = Jm + Jg + Js;
J2 = Jp2 + JL;
%% single loop controller
name = 'Single Loop';
% set up plant
single_loop_plant = Kvi * Kt * K*R1*R2*Rg ... 
    /(J2*(Rg^2*J1+Jp1)*s^4+K*(Rg^2*R2^2*J1+Jp1*R2^2+J2*R1^2)*s^2);
single_loop_plant = minreal(single_loop_plant);

% design controller
opt = pidtuneOptions("PhaseMargin", 60); % Default: 60 deg
single_loop_controller = pidtune(single_loop_plant, "PDF", 1, opt);

% design lead controller
% 10% OS -> zeta = 0.6
% zeta = .6;
% Ts = 1; % (s)
% 
% wn = 4 / (Ts * zeta); % (rad/s)
% zc = 80;
% 
% [~, ~, single_loop_controller] = design_lead_compensator(zeta, Ts, ...
%     zc, single_loop_plant, name);

% evaluate single loop controller
% 45 deg step
evaluate_controller(name, single_loop_controller, ...
    single_loop_plant, pi/4, 1000, "Position (rad)", "Voltage (V)");

% discretize controller
T = 0.005; %(s)
% save controller as .h file
fileID = fopen('../../C/Real Controllers/single_loop_controller.h','w');
slc2header(fileID, single_loop_controller, T, Kvi, Kt, BDI_per_rev);
%% double loop controllers
% the inner loop is nested inside the outer loop
%% inner loop controller
name = 'Inner Loop';
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

% z = 80; % Garbini's recommendation
zc = 120;

[~, ~, inner_loop_controller] = design_lead_compensator(zeta, Ts, ...
    zc, inner_loop_plant, name);

% evaluate inner loop controller
evaluate_controller(name, inner_loop_controller, ...
    inner_loop_plant, MCOT, .1, "Torque (N-m)", "Voltage (V)");
%% outer loop controller
name = 'Outer Loop';
% set up plant
Z2 = 1/(J2*s^2);
outer_loop_plant = series(feedback(series(inner_loop_controller, ...
    inner_loop_plant), 1), Z2);

% design controller
% output torque < 0.5 N-m
opt = pidtuneOptions("PhaseMargin", 60); % Default: 60 deg
outer_loop_controller = pidtune(outer_loop_plant, "PIDF", 4, opt);

% evaluate outer loop controller
% 45 deg step
evaluate_controller(name, outer_loop_controller, ...
    outer_loop_plant, pi/4, 10, "Position (rad)", "Torque (N-m)");
%% save double loop controller as .h file
% discretize inner loop controller
T = 0.005; %(s)

Krot = K*R1;
fileID = fopen('../../C/Real Controllers/double_loop_controller.h','w');
dlc2header(fileID, inner_loop_controller, outer_loop_controller, T, Krot, Kvi, Kt, BDI_per_rev)
%% close all open files
fclose('all');
%% functions
function [pc, Kc, controller] = design_lead_compensator(zeta, Ts, zc, plant, name)
    % this function designes a lead compensator for the given unity feedback system
    wn = 4 / (Ts * zeta); % (rad/s)
    tp = -zeta*wn + wn*sqrt(1-zeta^2)*1j; % target pole
    
    ps = pole(plant); % open loop poles
    [zs, Kp] = zero(plant); % open loop zeros
    
    ang = angle(tp + zc) - sum(angle(tp - ps)) + sum(angle(tp - zs));
    ang = mod((ang-pi), 2*pi);
    
    pc = imag(tp)/atan(ang) - real(tp); % controller pole
    Kc = (prod(abs(tp-ps))*abs(tp+pc))/(prod(abs(tp-zs))*abs(tp+zc))/Kp; % controller gain
    controller = zpk(-zc, -pc, Kc);

    % visualize controller design
    figure('NumberTitle', 'off', 'Name', append(name,' Controller Target vs Result'));
    open_loop_tf = series(controller, plant);
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
end

function [info, bw] = evaluate_controller(name, controller, plant, stepAmp, time, outputLabel, effortLabel)
    % plots graphs with information about a controller
    
    % output response
    figure('NumberTitle', 'off', 'Name', append(name, ' Output'));
    closed_loop = feedback(series(controller, plant), 1);
    opt = stepDataOptions('StepAmplitude', stepAmp); % 45 deg rotation step
    h = stepplot(closed_loop, time, opt);
    hold on
    y = yline(stepAmp);
    legend(y, 'Target')
    title("Output vs Time")
    ylabel(outputLabel)
    h.showCharacteristic('SettlingTime')
    h.showCharacteristic('PeakResponse')
    info = stepinfo(closed_loop);
    bw = bandwidth(closed_loop);

    % effort response
    figure('NumberTitle', 'off', 'Name', append(name, ' Effort'));
    controller_effort_transfer_function = feedback(tf(controller), plant);
    step(controller_effort_transfer_function, time, opt)
    title("Effort vs Time")
    ylabel(effortLabel)
end
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

% extra components
Jp1x = 3.528e-6; % (kg-m^2) shaft couplers, etc.
Jp2x = 1.398e-6; % (kg-m^2)

% motor encoder
BPRM = 500 * 4; % BDI Per Revolution Motor (BDI/rev)

% output encoder
% 2000 CPR * 4 ticks per window for a quadrature encoder
BPRL = 2000 * 4; % BDI Per Revolution Load (BDI/rev)

% pulleys
% pulley 1 (closest to the motor)
R1 = 2e-2; % (m) radius of the pulley closest to the motor
Jp1 = 2.871e-6; % (kg-m^2) moment of inertia of the pulley 
% pulley 2 (furthest from the motor)
R2 = R1; % (m) radius of the pulley furthest from the motor
Jp2 = Jp1; % (kg-m^2) moment of inertia of the pulley

% load
JL = .001981; % (kg-m^2) moment of inertia of the load

% springs
K1 = 578; % (N/m) spring constant of upper spring
K2 = K1; % (N/m) spring constant of lower spring

% amplifier
Kvi = 0.41; % (A/V) amplifier constant (from ME 477 lab)

% myRIO
maximum_output_voltage = 10; % (V) maximum output voltage of myRIO

% discretization
T = 0.005; %(s)
%% transfer function set up
s = tf("s");
K = K1 + K2;
J1 = Jm + Jg;
J2 = Jp2 + Jp2x + JL;
J3 = Jp1 + Jp1x;
% for controller header files
Krot = K*R2^2;
%% single loop controller
% simulations show controller is unstable
% in actuality, it might not be due to damping in the real system not
% present in the model
name = 'Single Loop';
% set up plant
single_loop_plant = Kvi * Kt * K*R1*R2*Rg ... 
    /(J2*(Rg^2*J1+J3)*s^4+K*(Rg^2*R2^2*J1+J3*R2^2+J2*R1^2)*s^2);
single_loop_plant = minreal(single_loop_plant);

% design controller
opt = pidtuneOptions("PhaseMargin", 60); % Default: 60 deg
single_loop_controller = pidtune(single_loop_plant, "PDF", 1, opt);

% evaluate single loop controller
[info, bw] = evaluate_controller(name, single_loop_controller, ...
    single_loop_plant, deg2rad(45), 1000, "Position (rad)", "Voltage (V)");

% visualize controller
% figure
% rlocus(series(single_loop_controller, single_loop_plant))
% hold on;
% rlocus(series(single_loop_controller, single_loop_plant), 1)

% save controller as .h file
fileID = fopen('../../C/Real Controllers/single_loop_controller.h','w');
ctrlrs2header(fileID, {single_loop_controller}, {'slc'}, T, Krot, Kvi, Kt, BPRM, BPRL, Rg)
%% double loop controllers
% the inner loop is nested inside the outer loop
%% inner loop controller
name = 'Inner Loop';
% set up plant
inner_loop_plant = Kvi * Kt * K*R1*R2*Rg/(K*R1^2+(J3+J1*Rg^2)*s^2);
inner_loop_plant = minreal(inner_loop_plant);

% design controller
% 10% OS -> zeta = 0.6
% zeta = .6; % Garbini's recommendation
zeta = .6;
Ts = 0.1;
zc = 30;

[~, ~, inner_loop_controller] = design_lead_compensator(zeta, Ts, ...
    zc, inner_loop_plant, name);
% 
% opt = pidtuneOptions("PhaseMargin", 80); % Default: 60 deg
% inner_loop_controller = pidtune(inner_loop_plant, "PDF")%, 40, opt);

% evaluate inner loop controller
% .1 N-m reference step
evaluate_controller(name, inner_loop_controller, ...
    inner_loop_plant, .1, .1, "Torque (N-m)", "Voltage (V)");
%% outer loop controller
name = 'Outer Loop';
% set up plant
Z2 = 1/(J2*s^2);
outer_loop_plant = series(feedback(series(inner_loop_controller, ...
    inner_loop_plant), 1), Z2);

% design controller
% output torque < 0.5 N-m
opt = pidtuneOptions("PhaseMargin", 60); % Default: 60 deg
outer_loop_controller = pidtune(outer_loop_plant, "PIDF", 7, opt);

% evaluate outer loop controller
evaluate_controller(name, outer_loop_controller, ...
    outer_loop_plant, deg2rad(45), 10, "Position (rad)", "Torque (N-m)");
%% save double loop controller as .h file
fileID = fopen('../../C/Real Controllers/double_loop_controller.h','w');
ctrlrs2header(fileID, {inner_loop_controller, outer_loop_controller}, {'ilc', 'olc'}, T, Krot, Kvi, Kt, BPRM, BPRL, Rg)
%% more evaluation of double loop controller
clamp = @(x, mn, mx) min(max(x,mn),mx); % x vector, minimum, maximum
tmax = 8;
ts = 0:T:tmax;
period = 4; %(s)
amp = .25;
lim = .2;
wave = square(2*pi/period/2*ts) .* (clamp(amp*sawtooth(ts*2*pi/period, 0.5), -lim/2, lim/2) + lim/2);
ylims = [-.3 .3];
figure('NumberTitle', 'off', 'Name', 'Reference Tracking Simulation');
% inner loop
subplot(2, 1, 1)
lsim(feedback(series(inner_loop_controller, inner_loop_plant),1), wave, ts)
title('Inner Loop Controller')
daspect([1 1 1])
ylim(ylims)
ylabel('Torque (N-m)')
% outer loop
subplot(2, 1, 2)
lsim(feedback(series(outer_loop_controller, outer_loop_plant),1), wave, ts)
title('Outer Loop Controller')
daspect([1 1 1])
ylim(ylims)
ylabel('Position (rad)')
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
    % grid on % sgrid is better
    sgrid(zeta,0)
    h2 = plot([tp conj(tp)], 'kx', 'MarkerSize',12);
    r = rlocus(open_loop_tf, 1);
    h1 = plot(r,'gs',    'MarkerEdgeColor','k',...
                        'MarkerFaceColor','g',...
                        'MarkerSize',9);
    legend([h1 h2] , ["closed loop poles" "target poles"])
end

function [info, bw] = evaluate_controller(name, controller, plant, ...
    stepAmp, time, outputLabel, effortLabel)
    % plots graphs with information about a controller
    figure('NumberTitle', 'off', 'Name', name);
    
    % Specified Equal Timescale Plots
    % output response
    subplot(2, 2, 1)
    closed_loop = feedback(series(controller, plant), 1);
    opt = stepDataOptions('StepAmplitude', stepAmp);
    h = stepplot(closed_loop, time, opt);
    hold on
    y = yline(stepAmp);
    legend(y, 'Target')
    title("Output vs Time (Specified Equal Timescale)")
    ylabel(outputLabel)
    h.showCharacteristic('SettlingTime')
    h.showCharacteristic('PeakResponse')
    info = stepinfo(closed_loop);
    bw = bandwidth(closed_loop);

    % effort response
    subplot(2, 2, 3)
    controller_effort_transfer_function = feedback(tf(controller), plant);
    step(controller_effort_transfer_function, time, opt)
    title("Effort vs Time (Specified Equal Timescale)")
    ylabel(effortLabel)
    
    % Scaled Timescale Plots
    % output response
    subplot(2, 2, 2)
    closed_loop = feedback(series(controller, plant), 1);
    opt = stepDataOptions('StepAmplitude', stepAmp);
    h = stepplot(closed_loop, opt);
    hold on
    y = yline(stepAmp);
    legend(y, 'Target')
    title("Output vs Time (Scaled Timescale)")
    ylabel(outputLabel)
    h.showCharacteristic('SettlingTime')
    h.showCharacteristic('PeakResponse')

    % effort response
    subplot(2, 2, 4)
    controller_effort_transfer_function = feedback(tf(controller), plant);
    step(controller_effort_transfer_function, opt)
    title("Effort vs Time (Scaled Timescale)")
    ylabel(effortLabel)
end
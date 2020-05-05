%% setup
close all; clc; % don't clear all, we need the variables from the design_controllers script
%% evaluate controller
maximum_motor_output_torque = maximum_output_voltage * Kvi * Kt
% output response
figure('NumberTitle', 'off', 'Name', 'Output');
closed_loop = feedback(series(controller, plant), 1);
opt = stepDataOptions('StepAmplitude', pi/4);
h = stepplot(closed_loop, opt);
title("Output vs Time")
ylabel("Position (rad)")
h.showCharacteristic('SettlingTime')
h.showCharacteristic('PeakResponse')
system_info = stepinfo(closed_loop)
system_bandwidth = bandwidth(closed_loop)
% effort response
figure('NumberTitle', 'off', 'Name', 'Effort');
inner_loop_controller_effort_transfer_function = feedback(tf(controller), plant);
step(inner_loop_controller_effort_transfer_function, opt)
title("Effort vs Time")
ylabel("Voltage (V)")
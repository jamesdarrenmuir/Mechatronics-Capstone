%% setup
close all; % don't clear all, we need the variables from the design_controllers script
%% evaluate single loop controller
% output response
figure('NumberTitle', 'off', 'Name', 'Single Loop Output');
closed_loop = feedback(series(single_loop_controller, plant), 1);
opt = stepDataOptions('StepAmplitude', pi/4); % 45 deg rotation step
h = stepplot(closed_loop, opt);
title("Output vs Time")
ylabel("Position (rad)")
h.showCharacteristic('SettlingTime')
h.showCharacteristic('PeakResponse')
single_loop_info = stepinfo(closed_loop)
single_loop_bandwidth = bandwidth(closed_loop)
% effort response
figure('NumberTitle', 'off', 'Name', 'Single Loop Effort');
controller_effort_transfer_function = feedback(tf(single_loop_controller), plant);
step(controller_effort_transfer_function, opt)
title("Effort vs Time")
ylabel("Voltage (V)")
%% evaluate double loop controllers
%% evaluate inner loop controller
% output response
figure('NumberTitle', 'off', 'Name', 'Inner Loop Output');
closed_loop = feedback(series(inner_loop_controller, KKSEA), 1);
maximum_motor_output_torque = maximum_output_voltage * Kvi * Kt;
opt = stepDataOptions('StepAmplitude', maximum_output_torque);
h = stepplot(closed_loop, opt);
title("Output vs Time")
ylabel("Torque (N-m)")
h.showCharacteristic('SettlingTime')
h.showCharacteristic('PeakResponse')
inner_loop_info = stepinfo(closed_loop)
inner_loop_bandwidth = bandwidth(closed_loop)
% effort response
figure('NumberTitle', 'off', 'Name', 'Inner Loop Effort');
controller_effort_transfer_function = feedback(tf(inner_loop_controller), KKSEA);
step(controller_effort_transfer_function, opt)
title("Effort vs Time")
ylabel("Voltage (V)")
%% evaluate outer loop controller
% output response
figure('NumberTitle', 'off', 'Name', 'Outer Loop Output');
closed_loop = feedback(series(outer_loop_controller, ZL), 1);
opt = stepDataOptions('StepAmplitude', pi/4); % 45 deg rotation step
h = stepplot(closed_loop, opt);
title("Output vs Time")
ylabel("Position (rad)")
h.showCharacteristic('SettlingTime')
h.showCharacteristic('PeakResponse')
outer_loop_info = stepinfo(closed_loop)
outer_loop_bandwidth = bandwidth(closed_loop)
% effort response
figure('NumberTitle', 'off', 'Name', 'Outer Loop Effort');
controller_effort_transfer_function = feedback(tf(outer_loop_controller), ZL);
step(controller_effort_transfer_function, opt)
title("Effort vs Time")
ylabel("Voltage (V)")
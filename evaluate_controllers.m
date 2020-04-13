%% setup
close all; clc; % don't clear all, we need the variables from the design_controllers script
%% evaluate inner loop controller
maximum_motor_output_torque = maximum_output_voltage * K_vi * K_t
% output response
figure
closed_inner_loop = feedback(series(inner_loop_controller, inner_loop_plant), 1);
opt = stepDataOptions('StepAmplitude', 0.9 * maximum_motor_output_torque);
h = stepplot(closed_inner_loop, opt);
title("Inner Loop Step Response")
ylabel("Torque (N-m)")
h.showCharacteristic('SettlingTime')
h.showCharacteristic('PeakResponse')
closed_inner_loop_info = stepinfo(closed_inner_loop)
% effort response
figure
inner_loop_controller_effort_transfer_function = feedback(tf(inner_loop_controller), inner_loop_plant);
step(inner_loop_controller_effort_transfer_function, opt)
title("Inner Loop Step Response Controller Effort")
ylabel("Voltage (V)")
%% evaluate outer loop controller
% output response
figure
closed_outer_loop = feedback(series(outer_loop_controller, Z_L), 1);
h = stepplot(closed_outer_loop);
title("Outer Loop Step Response")
ylabel("Position (rad)")
h.showCharacteristic('SettlingTime')
h.showCharacteristic('PeakResponse')
closed_outer_loop_info = stepinfo(closed_outer_loop)
% effort response
figure
outer_loop_controller_effort_transfer_function = feedback(tf(outer_loop_controller), Z_L);
step(outer_loop_controller_effort_transfer_function)
title("Outer Loop Step Response Controller Effort")
ylabel("Torque (N-m)")
%% evaluate entire system
figure
entire_system = feedback(outer_loop_controller * closed_inner_loop * Z_L, 1);
h = stepplot(entire_system);
title("System Step Response")
ylabel("Position (rad)")
h.showCharacteristic('SettlingTime')
h.showCharacteristic('PeakResponse')
system_info = stepinfo(entire_system)
system_bandwidth = bandwidth(entire_system)
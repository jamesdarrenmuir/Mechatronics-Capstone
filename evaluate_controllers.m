%% setup
close all; clc; % don't clear all, we need the variables from the design_controllers script
%% evaluate inner loop controller
closed_inner_loop = feedback(series(inner_loop_controller, SEA), 1);
h = stepplot(closed_inner_loop);
title("Inner Loop Step Response")
h.showCharacteristic('SettlingTime')
h.showCharacteristic('PeakResponse')
closed_inner_loop_info = stepinfo(closed_inner_loop)
% inner_loop_controller_effort_tf = feedback(tf(inner_loop_controller), SEA);
% step(inner_loop_controller_effort_tf)
% title("Inner Loop Step Response Controller Effort")
%% evaluate outer loop controller
closed_outer_loop = feedback(series(outer_loop_controller, Z_L), 1);
h = stepplot(closed_outer_loop);
title("Outer Loop Step Response")
h.showCharacteristic('SettlingTime')
h.showCharacteristic('PeakResponse')
closed_outer_loop_info = stepinfo(closed_outer_loop)
%% evaluate entire system
entire_system = feedback(outer_loop_controller * closed_inner_loop * Z_L, 1);
h = stepplot(entire_system);
title("System Step Response")
h.showCharacteristic('SettlingTime')
h.showCharacteristic('PeakResponse')
system_info = stepinfo(entire_system)
system_bandwidth = bandwidth(entire_system)
clearvars; close all; clc;
%% Inner Loop
% inner loop reference tracking
load("InnerLoop.mat")
f = figure('NumberTitle', 'off', 'Name', "Inner Loop Reference Tracking");
plot(time, reference_spring_torque, 'DisplayName', 'Reference Spring Torque');
hold on
plot(time, actual_spring_torque, 'DisplayName', 'Actual Spring Torque');
legend()
ylim([-.3 .3])
xlim([time(1) time(end)])
xlabel("Time (s)")
ylabel("Torque (N-m)")
title("Time vs Torque")
% print_to_PDF(f, 'inner_loop_reference_tracking')
%% Double Loop Disturbance
% double loop disturbance rejection
clearvars;
load("DoubleLoopDisturbance.mat")
f = figure('NumberTitle', 'off', 'Name', "Double Loop Disturbance Rejection");
n = 2; m = 1;
% position
subplot(n, m, 1);
plot(time, reference_position, 'DisplayName', 'Reference Position');
hold on
plot(time, actual_position, 'DisplayName', 'Actual Position');
legend()
% ylim([-.3 .3])
xlim([time(1) time(end)])
xlabel("Time (s)")
ylabel("Position (rad)")
title("Time vs Position")
% torque
subplot(n, m, 2);
plot(time, reference_spring_torque, 'DisplayName', 'Reference Spring Torque');
hold on
plot(time, actual_spring_torque, 'DisplayName', 'Actual Spring Torque');
legend()
% ylim([-.3 .3])
xlim([time(1) time(end)])
xlabel("Time (s)")
ylabel("Torque (N-m)")
title("Time vs Torque")
% print_to_PDF(f, 'double_loop_disturbance_rejection')
%% Double Loop Reference
% double loop reference tracking
clearvars;
load("DoubleLoopReference.mat")
f = figure('NumberTitle', 'off', 'Name', "Double Loop Reference Tracking");
n = 2; m = 1;
% position
subplot(n, m, 1);
plot(time, reference_position, 'DisplayName', 'Reference Position');
hold on
plot(time, actual_position, 'DisplayName', 'Actual Position');
legend()
% ylim([-.3 .3])
xlim([time(1) time(end)])
xlabel("Time (s)")
ylabel("Position (rad)")
title("Time vs Position")
% torque
subplot(n, m, 2);
plot(time, reference_spring_torque, 'DisplayName', 'Reference Spring Torque');
hold on
plot(time, actual_spring_torque, 'DisplayName', 'Actual Spring Torque');
legend()
% ylim([-.3 .3])
xlim([time(1) time(end)])
xlabel("Time (s)")
ylabel("Torque (N-m)")
title("Time vs Torque")
% print_to_PDF(f, 'double_loop_reference_tracking')
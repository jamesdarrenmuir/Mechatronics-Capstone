%% purpose
% quick and dirty visualization of collected data
%% setup
clear all; close all; clc;
addpath("data")
load("SEA.mat")
%% plot
figure('NumberTitle', 'off', 'Name', "SEA Data");
ys = {reference_position, actual_position, motor_torque, ...
    motor_position, reference_spring_torque, actual_spring_torque};
titles = {'load reference position', 'load actual position', 'motor torque (from output voltage)', ...
    'motor pulley position', 'reference spring torque', 'actual spring torque'};
ylabels = {'position (rad)', 'position (rad)', 'torque (N-m)', ...
    'position (rad)', 'torque (N-m)', 'torque (N-m)'};
numGraphs = length(ys);
for n=1:numGraphs
    subplot(numGraphs, 1, n)
    plot(time, ys{n})
    title(titles{n})
    xlabel("time (s)")
    ylabel(ylabels{n})
end
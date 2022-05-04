
clear; clc; close all;

addpath("../myFunctions/")

assignment3_acc; 
assignment4_acc; %keep in mind this doesn't clear variables

%Measured velocity - Kalman Filter velocity - Kalman Smoother velocity
fig = figure(100); plot(time, Euler_acc); hold on; plot(time, filter3_acc); hold on; plot(time, smoother_acc);
xlabel('Time [s]'); ylabel('Accelerations [rad/s^s]'); title("Kalman Filter vs Smoother acceleration estimation");
legend('Euler acceleration', 'Estimated acceleration (Kalman Filter)', 'Estimated acceleration (Kalman Smoother)');
set(fig, 'HandleVisibility', 'off');
close all;
set(fig, 'HandleVisibility', 'on');



clear; clc; close all;

addpath("../myFunctions/")

assignment3_vel;
assignment4_vel; %keep in mind this doesn't clear variables


%Measured velocity - Kalman Filter velocity - Kalman Smoother velocity
fig = figure(100); plot(time, M_vel); hold on; plot(time, filter_vel); hold on; plot(time, smoother_vel);
xlabel('Time [s]'); ylabel('Velocities [rad/s]'); title("Kalman Filter vs Smoother velocity estimation");
legend('Measured velocity', 'Estimated velocity (Kalman Filter)', 'Estimated velocity (Kalman Smoother)');
set(fig, 'HandleVisibility', 'off');
close all;
set(fig, 'HandleVisibility', 'on');


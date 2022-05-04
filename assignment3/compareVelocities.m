
clear; clc; close all;

assignment3_vel;
assignment3_acc; %keep in mind this doesn't clear variables

%Measured velocity - Kalman Filter velocity
fig = figure(100); plot(time, M_vel); hold on; plot(time, filter_vel); hold on; plot(time, filter3_vel);
xlabel('Time [s]'); ylabel('Velocities [rad/s]'); title("Kalman Filter velocity estimation");
legend('Measured velocity', 'Estimated velocity (x=2x1)', 'Estimated velocity (x=3x1)');
set(fig, 'HandleVisibility', 'off');
close all;
set(fig, 'HandleVisibility', 'on');


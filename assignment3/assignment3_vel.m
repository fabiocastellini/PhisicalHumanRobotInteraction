
clear; clc; close all;

%Add needed functions
addpath("../myFunctions/")

% fileID = fopen('../master_slave_500Hz.txt');
fileID = fopen('../master_slave_1kHz.txt');
% fileID = fopen('../master_slave_2kHz.txt');

dataFromFile = textscan(fileID,'%f %f %f %f %f %f %f %f','HeaderLines',1);
fclose(fileID);

Ts = 0.001; %1kHz measurements
time  = dataFromFile{1}; 
M_pos = dataFromFile{2}; M_vel = dataFromFile{3}; 
M_volt = dataFromFile{4};
time(1:292) = []; %cut samples until something happens (0.3s)
M_pos(1:292) = []; M_vel(1:292) = [];
M_volt(1:292) = [];

Euler_vel = (M_pos(2:end)-M_pos(1:(end-1))) / Ts;

%Show the velocity computed using the Euler approximation 
figure; plot(time(1:end-1), Euler_vel); hold on; plot(time(1:end-1), lowpass(Euler_vel,1,1/Ts),'LineWidth',1);
xlabel('Time [s]'); ylabel('Velocity [rad/s]'); title("Euler approximation to estimate velocity");
legend("Not filtered", "Filtered (lowpass)")

%Set the initial conditions
P_0 = diag([1e-5 1e-5]);
x_0 = [0 0].';

%Set the A, B and C matrices of the dynamic system
A = [1 Ts ; 
     0 1 ];
B = [Ts^2/2 ;
     Ts    ];
C = [1 0];
D = 0;

%% Kalman Filter
%Set the R and Q parameters
R = var(M_pos); %according to the theory, we keep it constant
q = 100000;
Q = q *[Ts^2/2; 
            Ts] * [Ts^2/2;
                     Ts]';
[filter_pos, filter_vel, ~, P_inf] = kalmanFilter(A, B, C, M_pos,R, Q, x_0, P_0);


%% Steady-state Kalman Filter
% Using the solution of the Algebric Riccati Equation            
% P_inf = A*P_inf*A.' - A*P_inf*C.'*inv(C*P_inf*C.' + R)*C*P_inf*A.' + Q;
[P_inf,K,L] = idare(A.', C.', Q, R);

%Compute the steady-state Kalman gain
K_inf = P_inf*C.'*inv(C*P_inf*C.' + R);

[filterSS_pos, filterSS_vel, ~, ~] = kalmanFilter(A, B, C, M_pos,R, Q, x_0, P_0, K_inf);


%% Kalman Predictor
[predictor_pos, predictor_vel, ~, ~] = kalmanPredictor(A, B, C, M_pos,R, Q, x_0, P_0);

%Compute the steady-state Kalman gain
Kbar_inf = A*P_inf*C.'*inv(C*P_inf*C.' + R);
[predictorSS_pos, predictorSS_vel, ~, ~] = kalmanPredictor(A, B, C, M_pos,R, Q, x_0, P_0, Kbar_inf);



%% Show the measured velocity against the estimated ones
% %Measured velocity - Kalman Filter velocity
% figure; plot(time, M_vel); hold on; plot(time, filter_vel); 
% xlabel('Time [s]'); ylabel('Velocities [rad/s]'); title("Kalman Filter velocity estimation");
% legend('Measured velocity', 'Estimated velocity');


% %Measured velocity - Kalman Predictor velocity
% figure; plot(time, M_vel); hold on; plot(time, predictor_vel); 
% xlabel('Time [s]'); ylabel('Velocities [rad/s]'); title("Kalman Predictor velocity estimation");
% legend('Measured velocity', 'Estimated velocity');

%Measured velocity - Kalman Filter velocity - Kalman Predictor velocity
figure; plot(time, M_vel); hold on; plot(time, filter_vel); hold on; plot(time, predictor_vel); 
xlabel('Time [s]'); ylabel('Velocities [rad/s]'); title("Kalman Filter velocity estimation");
legend('Measured velocity', 'Estimated velocity (filter)', 'Estimated velocity (predictor)');

%Measured velocity - Kalman Filter velocity - Kalman Filter s.s. velocity
figure; plot(time, M_vel); hold on; plot(time, filter_vel); hold on; plot(time, filterSS_vel);
xlabel('Time [s]'); ylabel('Velocities [rad/s]'); title("Kalman Filter velocity estimation at Steady-State");
legend('Measured velocity','Estimated velocity (filter)', 'Estimated velocity (filter at s.s.)');

%Measured velocity - Kalman Predictor velocity - Kalman Predictor s.s. velocity
figure; plot(time, M_vel); hold on; plot(time, predictor_vel); hold on; plot(time, predictorSS_vel);
xlabel('Time [s]'); ylabel('Velocities [rad/s]'); title("Kalman Predictor velocity estimation at Steady-State");
legend('Measured velocity','Estimated velocity (predictor)', 'Estimated velocity (predictor at s.s.)');


%Measured velocity - Kalman Filter velocity - Kalman Filter s.s. velocity
%Kalman Predictor velocity - Kalman Predictor s.s. velocity
figure; plot(time, M_vel); hold on; plot(time, filter_vel); hold on; plot(time, filterSS_vel);
hold on; plot(time, predictor_vel); hold on; plot(time, predictorSS_vel);
legend('Measured velocity','Estimated velocity (filter)', 'Estimated velocity (filter at s.s.)','Estimated velocity (predictor)', 'Estimated velocity (predictor at s.s.)');
xlabel('Time [s]'); ylabel('Velocities [rad/s]'); 
title("Comparison between all velocity estimations");


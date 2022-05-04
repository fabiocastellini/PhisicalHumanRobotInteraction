
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
figure; plot(time(1:end-1), Euler_vel); 
xlabel('Time [s]'); ylabel('Velocity [rad/s]'); title("Euler approximation to estimate velocity");

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
Q = 100;
[filter_pos, filter_vel, ~, P_inf] = kalmanFilter(A, B, C, M_pos,R, Q, x_0, P_0);


%% Steady-state Kalman Filter
Q = 100;

% Using the solution of the Algebric Riccati Equation            
% P_inf = A*P_inf*A.' - A*P_inf*C.'*inv(C*P_inf*C.' + R)*C*P_inf*A.' + Q;
[P_inf,K,L] = idare(A.', C.', Q, R);

%Compute the steady-state Kalman gain
K_inf = P_inf*C.'*inv(C*P_inf*C.' + R);

[filterSS_pos, filterSS_vel, ~, ~] = kalmanFilter(A, B, C, M_pos,R, Q, x_0, P_0, K_inf);


%% Kalman Predictor
Q = 100;
[predictor_pos, predictor_vel, ~, ~] = kalmanPredictor(A, B, C, M_pos,R, Q, x_0, P_0);

%Compute the steady-state Kalman gain
Kbar_inf = A*P_inf*C.'*inv(C*P_inf*C.' + R);
[predictorSS_pos, predictorSS_vel, ~, ~] = kalmanPredictor(A, B, C, M_pos,R, Q, x_0, P_0, Kbar_inf);


%% Verify with MATLAB (i don't understand how to make it work...i leave some code here though)

% sys = ss(A,[B B],C,D,Ts,'InputName',{'u' 'v'},'OutputName','y');  % Plant dynamics and additive input noise w

% A=[0 1
%    0 0];
% B=[0 ; 1 ];
% C=[0 1];
% D=[0]; 
% Plant = ss(A,B,C,D,Ts);
% Plant.InputName = 'un';
% Plant.OutputName = 'yt';
% 
% Sum = sumblk('un = u + w');
% sys = connect(Plant,Sum,{'u','w'},'yt');
% N=0;
% [kalmf,L,P] = kalman(sys,Q,R,N);
% size(kalmf)
% kalmf.InputName
% kalmf.OutputName
% kalmf = kalmf(1,:);
% 
% sys.InputName = {'u','w'};
% sys.OutputName = {'yt'};
% vIn = sumblk('y=yt+v');
% 
% kalmf.InputName = {'u','y'};
% kalmf.OutputName = 'ye';
% 
% SimModel = connect(sys,vIn,kalmf,{'u','w','v'},{'yt','ye'});
% % t = (0:100)';
% % u = sin(t/5);
% % rng(10,'twister');
% t = time;
% u = M_pos;
% w = 0.0001*sqrt(Q)*randn(length(t),1);
% v = 0.0001*sqrt(R)*randn(length(t),1);
% out = lsim(SimModel,[u,w,v]);
% yt = out(:,1);   % true response
% ye = out(:,2);  % filtered response
% y = yt + v;     % measured response
% clf
% subplot(211), plot(t,M_vel,t,yt,'b',t,ye,'r--'), 
% xlabel('Number of Samples'), ylabel('Output')
% title('Kalman Filter Response')
% legend('M_vel','True','Filtered')
% subplot(212), plot(t,yt-y,'g',t,yt-ye,'r--'),
% xlabel('Number of Samples'), ylabel('Error')
% legend('True - measured','True - filtered')

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


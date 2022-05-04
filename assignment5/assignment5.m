

%% Choose the data!

assignment3_vel;
assignment3_acc; %doens't clear variables
assignment4_vel; %doens't clear variables
assignment4_acc; %doens't clear variables

Y = M_volt; %voltage measurements

X = [filter3_acc, filter_vel]; %estimated acceleration and velocity
% X = [smoother_acc, smoother_vel]; %estimated acceleration and velocity

Euler_vel = (M_pos(2:end)-M_pos(1:end-1))./Ts;
Euler_vel = lowpass(Euler_vel, 3, 1/Ts);
Euler_vel(end+1) = Euler_vel(end);

Euler_acc = (M_vel(2:end)-M_vel(1:end-1))./Ts;
Euler_acc = lowpass(Euler_acc, 3, 1/Ts);
Euler_acc(end+1) = Euler_acc(end);

% X = [Euler_acc, Euler_vel];



%% Least Square estimation 
beta_hat = inv(X.'*X)*X.' * Y;   %parameters estimation

J_LS = beta_hat(1);    %motor's inertia
k_LS = 1/beta_hat(2);
d_LS = beta_hat(2);    %motor's damping

disp("-- Least Square estimation --")
fprintf("The estimated inertia is: %.5f kg*m^2\nThe estimated damping is: %.5f kg*m^2/s\n", J_LS,d_LS);
disp("-----------------------------")

X_hat = Y * pinv(beta_hat);
X_hat = lowpass(X_hat, 5, 1/Ts); %needed to remove noise
figure; plot(time, X_hat(:,2));   hold on; plot(time, X(:,2), 'LineWidth',2);
title("LS estimation - velocity reprojection");
legend('Reprojected velocity, after computing theta-hat', 'Estimated velocity (Kalman)');
figure; plot(time, X_hat(:,1)); hold on; plot(time, X(:,1), 'LineWidth',2);
title("LS estimation - acceleration reprojection");
legend('Reprojected acceleration, after computing theta-hat','Estimated acceleration (Kalman)');



%% Recursive Least Square estimation 

lambda = 0.9999; %forgetting factor

%Initialization
error = zeros(size(X, 1),1); 
clear K beta_hat
for i=1:size(X, 1)
    Xk{i} = X(i,:);           %1x2
    P{i} = eye(2);            %2x2
    K{i} = zeros(2,1);        %2x1
    beta_hat{i} = zeros(2,1);  %2x1
end
 
for k=2:(size(X, 1))   
  
   %Compute the prediction error
   error(k) = Y(k) - Xk{k} * beta_hat{k-1};

   %Compute current parameters' estimates
%    K = (P{k-1} * Xk{k}.') / (lambda + Xk{k}*P{k-1}*Xk{k}.'); %according to some sources
   K = (P{k-1} * Xk{k}.'); %according to the slides

   beta_hat{k} = beta_hat{k-1} + K*error(k,1);      
 
   P{k} = 1/lambda*( P{k-1} - (P{k-1} * Xk{k}.' * Xk{k} *P{k-1})/(lambda + Xk{k}*P{k-1}*Xk{k}.') ); 
end
beta_hat = beta_hat{end};
J_RLS = beta_hat(1);     %motor's inertia
k_RLS = 1/beta_hat(2);
d_RLS = beta_hat(2);     %motor's damping

disp("-- Recursive Least Square estimation --")
fprintf("The estimated inertia is: %.5f kg*m^2\nThe estimated damping is: %.5f kg*m^2/s\n", J_RLS,d_RLS);
disp("-----------------------------")

X_hat = Y * pinv(beta_hat);
X_hat = lowpass(X_hat, 5, 1/Ts); %needed to remove noise

figure; plot(time, X_hat(:,2)); hold on; plot(time, X(:,2), 'LineWidth',2);
title("RLS estimation - velocity reprojection");
legend('Reprojected velocity, after computing theta-hat', 'Estimated velocity (Kalman)');

figure; plot(time, X_hat(:,1)); hold on; plot(time, X(:,1), 'LineWidth',2);
title("RLS estimation - acceleration reprojection");
legend('Reprojected acceleration, after computing theta-hat','Estimated acceleration (Kalman)');


%% Adaptive algorithm 
clear beta_hat

Ts = time(2) - time(1);
g = 0.005; %???

e(1) = Y(1) - Xk{1} * [0; 0];
beta_hat{1} = Ts * g * Xk{1}.' * e(1);

for k=2:size(X,1)
    error(k) = Y(k) - Xk{k} * beta_hat{k-1};
   
    beta_hat{k} = beta_hat{k-1} + g * Ts * Xk{k}.' * error(k);
end

beta_hat = beta_hat{end};
J_ADAPTIVE = beta_hat(1);     %motor's inertia
k_ADAPTIVE = 1/beta_hat(2);
d_ADAPTIVE = beta_hat(2);     %motor's damping

disp("-- Adaptive algorithm (dicrete time) --")
fprintf("The estimated inertia is: %.5f kg*m^2\nThe estimated damping is: %.5f kg*m^2/s\n", J_ADAPTIVE,d_ADAPTIVE);
disp("-----------------------------")

X_hat = Y * pinv(beta_hat);
X_hat = lowpass(X_hat, 5, 1/Ts); %needed to remove noise

figure; plot(time, X_hat(:,2)); hold on; plot(time, X(:,2), 'LineWidth',2);
title("Adaptive discrete estimation - velocity reprojection");
legend('Reprojected velocity, after computing theta-hat', 'Estimated velocity (Kalman)');

figure; plot(time, X_hat(:,1)); hold on; plot(time, X(:,1), 'LineWidth',2);
title("Adaptive discrete estimation - acceleration reprojection");
legend('Reprojected acceleration, after computing theta-hat','Estimated acceleration (Kalman)');



%% "Manual tuning of beta"
% close all;

X_hat = Y * pinv([0.08 0.1].');
X_hat = lowpass(X_hat, 5, 1/Ts); %needed to remove noise

% mse(X, X_hat)

figure; plot(time, X_hat(:,2)); hold on; plot(time, X(:,2), 'LineWidth',2);
hold on; plot(time, M_vel, 'LineWidth',2);
title("Manual tuning (velocity)");
legend('Reprojected velocity, after imposing theta-hat', 'Estimated velocity (Kalman)', 'Measured velocity');

figure; plot(time, X_hat(:,1)); hold on; plot(time, X(:,1), 'LineWidth',2);
title("Manual tuning (acceleration)");
legend('Reprojected acceleration, after imposing theta-hat','Estimated acceleration (Kalman)');


%To test the above estimations in Simulink:
open_system('dcMotor');
load_system('dcMotor')
model = 'dcMotor';

%Run the simulation and get the outputs
in = Simulink.SimulationInput(model);
out = sim(in);





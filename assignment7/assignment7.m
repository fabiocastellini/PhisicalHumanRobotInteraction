
clear; clc; close all;

%Add needed functions
addpath("../myFunctions/")

set(cstprefs.tbxprefs,'FrequencyUnits','Hz','Grid','on')
s = tf('s');


clear; clc; close all;

%Add needed functions
addpath("../myFunctions/")

set(cstprefs.tbxprefs,'FrequencyUnits','Hz','Grid','on')
s = tf('s');

%-------- Set paramters (ACCORDING to 24-01-22 lecture) --------%
%Robots - mass, damping parameters 
Mm = 0.5; %master robot's mass
Dm = 0;   %master robot's damping
Ms = 2;   %slave robot's mass
Ds = 0;   %slave robot's damping 

%Human - impedence of the human operator's arm
Jh = 0; %0.5;  %inertia coefficient
Bh = 1.5; %70; %damping coefficient
Kh = 1; %2000; %stiffness coefficient

%Environment inertia, damping, spring parameters
Je = 0;   %environment inertia (we interact with a static deformable environment)
Be = 10;  %environment damping
Ke = 200; %environment spring/stiffness
%--------------------------------------------------------------%

%Desired reference position signal
Amplitude = 1; %sine/step amplitude
Fc = 0.1;      %sine frequency (Hz)
Flp = 5;       %step low pass filter cutoff frequency

%Sampling time
Ts = 0.001; 

%Noise variance on the position+forces measurements
noiseVariance = 0.00001;

%Use Kalman Filter to estimate velocities 
A_kalman = [1 Ts ; 0 1 ]; B_kalman = [Ts^2/2 ; Ts]; C_kalman = [1 0];
q_master = 1000000;
q_slave = 1000000;
Q_master = q_master * ( B_kalman * B_kalman' );
Q_slave = q_slave * ( B_kalman * B_kalman' );
R = 1;

%Two-layer architecture parameters
alpha = 0.1; %needed for the TLC
beta = 0.01;
Hd = 1;
H0_master = 1;
H0_slave = 1;

%Controller - human intention model coefficients
Dh = 2;  %Derivative
k_h = 5; %Proportional


%% Parameters that must be choosen:
delay = 20;  %instants of delays

scheme = input("Choose the scheme: F-P or P-P? (0/1)                             ");
noise = input("Do you want a noise free or noisy (+Kalman) simulation? (0/1)    ");
%% Controller - human intention model coefficients (Ch=PID/PD/PI depends on the reference signal)
contact = input("Do you want to simulate in contact with the environment? (0/1)   ");
if contact == 0
    xe = 1; %environment position (NO INTERACTION)
    if scheme == 0
        %Good parameters in free motion F-P:
        Dh = 1;   %Derivative position controller coefficient
        k_h = 3; %Proportional position controller coefficient
        Ih = 2;   %Integrative position controller coefficient
    elseif scheme == 1
        %Good parameters in free motion P-P:
        Dh = 2;   %Derivative position controller coefficient
        k_h = 5; %Proportional position controller coefficient
        Ih = 2;   %Integrative position controller coefficient
    end
elseif contact == 1
    xe = 0.65; %environment position (INTERACTION)
    if scheme == 0
        %Good parameters in contact F-P:
        Dh = 1;   %Derivative position controller coefficient
        k_h = 2; %Proportional position controller coefficient
        Ih = 0;   %Integrative position controller coefficient
    elseif scheme == 1
        %Good parameters in contact P-P:
        Dh = 2;   %Derivative position controller coefficient
        k_h = 2; %Proportional position controller coefficient
        Ih = 0;   %Integrative position controller coefficient
    end
end
  

%Load and open the Simulink system
%% FORCE - POSITION discrete model + NOISE and KALMAN FILTERING:
if scheme == 0    
    Bs = 40; %slave robot velocity controller parameters
    Ks = 50; %slave robot velocity controller parameters
    open_system('siso_4_channel_teleop_discrete_tank_forcePos_noise');
    load_system('siso_4_channel_teleop_discrete_tank_forcePos_noise')
    model = 'siso_4_channel_teleop_discrete_tank_forcePos_noise';
    
    
    %% To activate NOISE and KALMAN FILTERING:
    if noise == 1
        set_param('siso_4_channel_teleop_discrete_tank_forcePos_noise/encoder + Kalman estimator/swNOISE1','sw','1')
        set_param('siso_4_channel_teleop_discrete_tank_forcePos_noise/encoder + Kalman estimator /swNOISE2','sw','1')
        set_param('siso_4_channel_teleop_discrete_tank_forcePos_noise/swNOISE3','sw','1')
    elseif noise == 0
        %% To use NOISE FREE simulation:
        set_param('siso_4_channel_teleop_discrete_tank_forcePos_noise/encoder + Kalman estimator/swNOISE1','sw','0')
        set_param('siso_4_channel_teleop_discrete_tank_forcePos_noise/encoder + Kalman estimator /swNOISE2','sw','0')
        set_param('siso_4_channel_teleop_discrete_tank_forcePos_noise/swNOISE3','sw','0')
    end
elseif scheme == 1
    %% POSITION - POSITION discrete model + NOISE and KALMAN FILTERING:
    Bs = 20; %slave robot velocity controller parameters
    Ks = 25; %slave robot velocity controller parameters
    Bm = 15; %master robot velocity controller parameters
    Km = 5; %master robot velocity controller parameters
    open_system('siso_4_channel_teleop_discrete_tank_PosPos_noise');
    load_system('siso_4_channel_teleop_discrete_tank_PosPos_noise')
    model = 'siso_4_channel_teleop_discrete_tank_PosPos_noise';
    
        
    %% To activate NOISE and KALMAN FILTERING: 
    if noise == 1
        set_param('siso_4_channel_teleop_discrete_tank_PosPos_noise/encoder + Kalman estimator/swNOISE1','sw','1')
        set_param('siso_4_channel_teleop_discrete_tank_PosPos_noise/encoder + Kalman estimator /swNOISE2','sw','1')
        set_param('siso_4_channel_teleop_discrete_tank_PosPos_noise/swNOISE3','sw','1')
    elseif noise == 0
        %% To use NOISE FREE simulation:
        set_param('siso_4_channel_teleop_discrete_tank_PosPos_noise/encoder + Kalman estimator/swNOISE1','sw','0')
        set_param('siso_4_channel_teleop_discrete_tank_PosPos_noise/encoder + Kalman estimator /swNOISE2','sw','0')
        set_param('siso_4_channel_teleop_discrete_tank_PosPos_noise/swNOISE3','sw','0')
    end
end


%Run the simulation and get the outputs
in = Simulink.SimulationInput(model);
out = sim(in);






fprintf("\n\n-- Summary: --\n");
if scheme
    disp("- Position-Position architecture")
else
    disp("- Force-Position architecture")
end
if noise
    disp("- Noisy position and force measurements + Kalman estimator")
else
    disp("- Noise free position and force measurements")
end
if contact
    disp("- Contact with the environment")
else
    disp("- Free motion")
end


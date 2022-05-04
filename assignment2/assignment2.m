
clear; clc; close all;

set(cstprefs.tbxprefs,'FrequencyUnits','Hz','Grid','on')
s = tf('s');
Ts = 0.001;

%-------- Given data --------%
%Robots - mass parameters 
Mm = 0.5; %master robot's mass
Ms = 2;   %slave robot's mass
Dm = 0;  %master damping - try with 5
Ds = 0;  %slave damping  - try with 10

%Human - impedence of the human operator's arm
% Jh = 0.5;  %inertia coefficient
% Bh = 70;   %damping coefficient
% Kh = 2000; %stiffness coefficient
Jh = 0.05;  %inertia coefficient
Bh = 7;   %damping coefficient
Kh = 200; %stiffness coefficient
%Zh = Jh*s + Bh + Kh/s; %t.f.

%Environment parameters
Je = 0;   %we interact with a static deformable environment
Be = 100; %environment damping
Ke = 200; %environment spring/stiffness
xe = 1; %environment position

%Desired reference position signal
A = pi/4; %sinusoidal amplitude
Fc = 1;   %sinusoidal frequency
Flp = 5; %low pass filter cutoff frequency



%-------- Set paramters --------%
%Controller - master robot controller parameters
Bm = 100;    %master controller's Derivative
Km = 500;   %master controller's Proportional

%Controller - human model coefficients
Dh = 500;   %Derivative
k_h = 8000; %Proportional

%Controller - slave robot controller parameters
Bs = 1000;  %slave controller's Derivative 
Ks = 80000; %slave controller's Proportional
% Bs = Bm*4;  %slave controller's Derivative 
% Ks = Km*4; %slave controller's Proportional

Cmf = -0.67; %constant force master controller
Csf = -0.67; %constant force slave controller


%- Transfer functions -%
Cm = Bm + Km/s; %master controller's t.f.
Cs = Bs + Ks/s; %slave controller's t.f.
Zm = Mm*s + Dm; Zm_inv = 1/Zm;
Zs = Ms*s + Ds; Zs_inv = 1/Zs;


%% Load and open the Simulink system
open_system('siso_4_channel_teleop_force');
load_system('siso_4_channel_teleop_force')
model = 'siso_4_channel_teleop_force';

%Run the simulation and get the outputs
in = Simulink.SimulationInput(model);
out = sim(in);


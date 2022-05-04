
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
Ke = 2000; %environment spring/stiffness
xe = 0.4; %environment position -> contact
% xe = 1; %environment position -> free motion

%Desired reference position signal
A = pi/4; %sinusoidal amplitude
Fc = 1;   %sinusoidal frequency
Flp = 5; %low pass filter cutoff frequency



%-------- Set paramters --------%


%Controller - master robot controller parameters
Bm = 100;    %master controller's Derivative
Km = 400;   %master controller's Proportional

%Controller - human model coefficients
Dh = 100;   %Derivative
k_h = 40000; %Proportional

%Controller - slave robot controller parameters
Bs = 1000;  %slave controller's Derivative 
Ks = 80000; %slave controller's Proportional
% Bs = Bm*4;  %slave controller's Derivative 
% Ks = Km*4; %slave controller's Proportional

%- Transfer functions -%
Cm = Bm + Km/s; %master controller's t.f.
Cs = Bs + Ks/s; %slave controller's t.f.
Zm = Mm*s + Dm; Zm_inv = 1/Zm;
Zs = Ms*s + Ds; Zs_inv = 1/Zs;


%Hybrid matrix imposing perfect transparency
H_bar = [0  1; 
         -1 0]; 
H11 = H_bar(1,1); H12 = H_bar(1,2); 
H21 = H_bar(2,1); H22 = H_bar(2,2);  

%Environment impedence (check that Zt_width = inf)
Ze = Je*s + Be + Ke/s;
Zt = (H11 - H12*Ze) / (H21 - H22*Ze);
if isequal(zpk(Zt), zpk(Ze))
    fprintf("Zt = Ze in the current Hybrid matrix configuration!\n");
else
    fprintf("Zt != Ze in the current Hybrid matrix configuration!\n");
end
Zt_width = H12/H22 - H11/H21; %imposing Ze = 0
fprintf("Zt_width = %f\n", Zt_width);
    

%Load and open the Simulink system
% open_system('siso_4_channel_teleop_continuous');
% load_system('siso_4_channel_teleop_continuous')
% model = 'siso_4_channel_teleop_continuous';
open_system('siso_4_channel_teleop_digital');
load_system('siso_4_channel_teleop_digital')
model = 'siso_4_channel_teleop_digital';

%Run the simulation and get the outputs
in = Simulink.SimulationInput(model);
out = sim(in);


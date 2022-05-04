clear all;

% Input sin parameter 
A = 0.5;
Fc = 1; 


% Human impedance parameters
Jh = 0.05;%0.5;
Bh = 1.5;%70;
Kh = 0;%200;

% Impedance master/slave
m_master = 0.5;
d_master = 0.1;

m_slave = 2;
d_slave = 0.4;

% Controller master
k_master = 0;%1000;
b_master = 0;%100;


% Controller slave
k_slave = 0; %k_master*4;
b_slave = 5;  %b_master*4;

% Local force loop controller 
% -0.67 5th section paper


% Human intention controller (PI)
Ph = 5;
Dh = 2;

% Wave variables constant

b = 1;

% Environment impedance parameters
Je = 0;
Be = 10; 
Ke = 200;
env_pos = 10;

Ts = 0.001;

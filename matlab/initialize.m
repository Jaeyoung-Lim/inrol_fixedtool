clear all;
clc;

% I set the simulink with time step of 0.001, gravity is 9.81

% control gain
b = 10;
k = 15;
k_w = 0;
k_w_d = 0;

% trajectory parameters
w=-1;
w1 = 0.5;
w21 = 0.3;
w22 = -0.3;

% system parameters
m=0.5;
jx = 0.02;
jy = 0.02;
jz = 0.01;

d1 = 0.4;
d2 = 0; 
d3 = -0.3;


mex toc_con.cpp  % use this if toc_con.cpp is updated
msg = 'go'

save('parameters.mat');



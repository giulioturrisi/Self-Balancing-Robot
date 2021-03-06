function [u_l,u_r] = sliding_mode_fun(state, state_d)
%SLIDING_MODE_FUN Summary of this function goes here
%   Detailed explanation goes here
c = 200;
s = c*(state(3) - state_d(3)) + state(4);

%M = 1.426971; %kg
M = 5;
com_pos = [0 0 10.007]; %mm
%M_w = 0.1;
M_w = 0.46;
%J_theta = 0.005928; %kg m^2
J_theta = 2.167e-02;
J_phi = 4.167e-03;
%J_w = 0.001;
J_w = 4.968e-03
%J_phi = 0.003343;
r = 0.2;
d = 0.2; %mm
g = 9.8;
l =0.8;

a = M + 2*M_w + 2*J_w/r^2;
u_eq = r*(a*M*g*l*sin(state(3)) + c*((state(3) - state_d(3)))*(J_phi*a - M^2*l^2*cos(state(3))^2))/(2*M*l*cos(state(3)));
u_disc = 1.0*tanh(s);

u_l = (u_eq - u_disc);
u_r = (u_eq - u_disc);
end


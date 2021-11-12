function [theta_ddot,phi_ddot,x_ddot] = forward_dynamic_fun(u_l,u_r,state)
%forward dynamics

%robot parameters
M = 1.426971; %kg
%M = 5;
com_pos = [0 0 10.007]; %mm
%M_w = 0.1;
M_w = 0.46;
%J_theta = 0.005928; %kg m^2
J_theta = 2.167e-02;
J_phi = 4.167e-03;
%J_w = 0.001;
J_w = 4.968e-03;
%J_phi = 0.003343;
r = 0.2;
d = 0.2; %mm
g = 9.8;
l =0.8;

x = state(1);
x_dot = state(2);
theta = state(3);
thet_dot = state(4);
phi = state(5);
phi_dot = state(6);

theta_ddot = ((M + 2*M_w + 2*J_w/r^2)*(M*g*l*sin(theta)) - M*l*cos(theta)*(u_l + u_r)/r)/((J_phi)*(M + 2*M_w + 2*J_w/r^2) - M^2*l^2*cos(theta)*cos(theta));
phi_ddot = (d/2)*((u_l - u_r)/r)/(J_phi + ((d^2)/(2*(J_w/r^2 + M_w))));
x_ddot = ((u_l + u_r)/r - M*l*cos(theta)*(M*g*l*sin(theta))/J_theta)/(M + 2*M_w + 2*J_w/r^2 - (M^2*l^2*cos(theta)^2)/J_theta);       
end


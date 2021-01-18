function [A,B] = linearization_fun(u_l,u_r,theta,theta_dot,phi,phi_dot)
%LINEARIZATION_FUN Summary of this function goes here
%   Detailed explanation goes here
M = 1.426971; %kg
com_pos = [0 0 10.007]; %mm
M_w = 0.1;
J_theta = 0.005928; %kg m^2
J_w = 0.001;
J_phi = 0.003343;
r = 0.1;
d = 0.7; %mm
g = 9.8;
l =0.8;

%theta_ddot = (M + 2*M_w + 2*J_w/r^2)*(M*g*l*sin(theta)) - M*l*cos(theta)*(u_l + u_r)/R)/((J)*(M + 2*M_w + 2*J_w/r^2) - M^2*l^2*cos(theta));
%phi_ddot = (d/2)*((u_l - u_r)/r)/(J_phi + ((d^2)/(2*(J_w/R^2) + M_w)));
%x_ddot = ((u_l + u_r)/R - M*l*cos(theta)*(M*g*l*sin(theta))/J_theta)/(M + 2*M_w + 2*J_w/r^2 - (M^2*l^2*cos(theta)^2)/J_theta);


%linearization with state theta, theta_dot, phi, phi_dot
a = M + 2*M_w + 2*J_w/r^2;
b = (d/2)*r/(J_phi + (d^2/r)*((J_w/r^2)+M_w));
A = [0 1 0 0; a^2*M*g*l*J_theta 0 0 0; 0 0 0 1; 0 0 0 0];
B = [0 0; M*l/r M*l/r; 0 0; b -b];
end


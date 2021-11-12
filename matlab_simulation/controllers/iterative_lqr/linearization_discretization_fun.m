function [A,B] = linearization_discretization_fun(u_l,u_r,theta,theta_dot,phi,phi_dot)
%Linearization over desired point

%robot parameters
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


%linearization with state theta, theta_dot, phi, phi_dot - to do other lin. point
%a = M + 2*M_w + 2*J_w/r^2;
%b = (d/2)/r*(J_phi + (d^2/r)*((J_w/r^2)+M_w));
%A = [0 1 0 0; a^2*M*g*l*J_theta 0 0 0; 0 0 0 1; 0 0 0 0];
%B = [0 0; M*l/r M*l/r; 0 0; b -b];

a = M + 2*M_w + 2*J_w/r^2;
num = a*M*g*l*sin(theta) - M*l*cos(theta)*(u_l + u_r)/r;
den = (J_theta*a - M^2*l^2*cos(theta)^2);

theta_ddot = (a*M*g*l*cos(theta) + M*l*sin(theta)*(u_l + u_r)/r)*den - M^2*l^2*sin(2*theta)*num;

A_1 = [0 1 0 0; theta_ddot/den^2 0 0 0];
A_2 = [0 0 0 1; 0 0 0 0];
A = vertcat(A_1,A_2);

b = (d/2)*r/(J_phi + d^2/(2*(J_w/r^2+M_w)));
B = [0 0; -(M*l*cos(theta)/(r*den))*(1), -(M*l*cos(theta)/(r*den))*(1); 0 0; b*(1) , b*(- 1)];


%discretize
B = pinv(eye(4) - A*0.01/2.)*B*sqrt(0.01);
A = (eye(4) + A*0.01/2.)*pinv(eye(4) - A*0.01/2.);

end


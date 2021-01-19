%theta_l roation angle wheel left
%theta_r roation angle wheel right
%theta angle pendulum
%M mass chassis
%M_w mass wheel
%phi angle steering
%r radius wheel
%d distance wheel axis - com axis
%l distance com - axis wheel
%J_w inertia wheel
%J_theta inertia chassis along theta
%J_theta inertia chassis along phi

M = 1.426971; %kg
com_pos = [0 0 10.007] %mm
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
den = (J_theta*a - M^2*l^2*cos(0)^2);
A = [0 1 0 0; a^2*M*g*l*J_theta 0 0 0; 0 0 0 1; 0 0 0 0];
B = [0 0; -M*l/(r*den) -M*l/(r*den); 0 0; b -b];
Q = eye(4);
R = eye(2);
lqr(A,B,Q,R,0)

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

%theta_ddot = (M + 2*M_w + 2*J_w/r^2)*(M*g*l*sin(theta)) - M*l*cos(theta)*(u_l + u_r)/R)/((J)*(M + 2*M_w + 2*J_w/r^2) - M^2*l^2*cos(theta));
%phi_ddot = (d/2)*((u_l - u_r)/r)/(J_phi + ((d^2)/(2*(J_w/R^2) + M_w)));
%x_ddot = ((u_l + u_r)/R - M*l*cos(theta)*(M*g*l*sin(theta))/J_theta)/(M + 2*M_w + 2*J_w/r^2 - (M^2*l^2*cos(theta)^2)/J_theta);


%linearization with state theta, theta_dot, phi, phi_dot
% a = M + 2*M_w + 2*J_w/r^2;
% b = (d/2)*r/(J_phi + (d^2/r)*((J_w/r^2)+M_w));
% den = (J_theta*a - M^2*l^2*cos(0)^2);
% A = [0 1 0 0; a*M*g*l*(J_theta*a - M^2*l^2)/den^2 0 0 0; 0 0 0 1; 0 0 0 0];
% 
% 
% B = [0 0; -M*l/(r*den) -M*l/(r*den); 0 0; b -b];
% 
% Q = eye(4);
% R = eye(2);
% [K,s,e] = lqr(A,B,Q,R,0)

%A = A*0.0001 + eye(4);
%B = B*0.0001;

% B = pinv(eye(4) - A*0.01/2.)*B*sqrt(0.01);
% A = (eye(4) + A*0.01/2.)*pinv(eye(4) - A*0.01/2.);
% 
% [K,s,e] = dlqr(A,B,Q,R,0)



% %lin around forced eq
a = M + 2*M_w + 2*J_w/r^2;

theta=-0.0;
theta_dot=0;
phi=0.0;
phi_dot=0;

cos_theta = cos(theta);
sin_theta = sin(theta);
sin_2theta = sin(2*theta);



u_l = (r/2)*(a*M*g*l*sin(theta))/(M*l*cos(theta))
%u_l = 0;
u_r=u_l;
u_ff = u_r;


num = a*M*g*l*sin(theta) - M*l*cos(theta)*(u_l + u_r)/r
den = (J_theta*a - M^2*l^2*cos(theta)^2);

theta_ddot = (a*M*g*l*cos(theta) + M*l*sin(theta)*(u_l + u_r)/r)*den - M^2*l^2*sin(2*theta)*num;

A_1 = [0 1 0 0; theta_ddot/den^2 0 0 0];
A_2 = [0 0 0 1; 0 0 0 0];


A = vertcat(A_1,A_2);
b = (d/2)*r/(J_phi + d^2/(2*(J_w/r^2+M_w)));
B = [0 0; -(M*l*cos(theta)/(r*den))*(1), -(M*l*cos(theta)/(r*den))*(1); 0 0; b*(1) , b*(- 1)];

Q = eye(4)*2;
R = eye(2)*0.1;
%[K,s,e] = lqr(A,B,Q,R,0)


B_eq = pinv(eye(4) - A*0.01/2.)*B*sqrt(0.01);
A_eq = (eye(4) + A*0.01/2.)*pinv(eye(4) - A*0.01/2.);

[k_lqr,P_f,e] = dlqr(A_eq,B_eq,Q,R,0);


%with X
num_x = (u_l + u_r)/r - M*l*cos(theta)*M*g*l*sin(theta)/J_theta;
den_x = a - (M^2*l^2*cos(theta)^2)/J_theta;
%x_ddot = (M^2*l^2*g*(sin(theta)^2 - cos(theta)^2)/J_theta)*den_x - (M*l^2*sin(2*theta))*num_x/J_theta
x_ddot = -(M^2*l^2*g*(cos(2*theta))/J_theta)*den_x - (M*l^2*sin(2*theta))*num_x/J_theta



A_0 = [0 1 0 0 0 0; 0 0 x_ddot/(den_x^2) 0 0 0];
A_1 = [0 0 0 1 0 0; 0 0 theta_ddot/den^2 0 0 0];
A_2 = [0 0 0 0 0 1; 0 0 0 0 0 0];
A = vertcat(A_1,A_2);
A_withx = vertcat(A_0,A);
b = (d/2)*r/(J_phi + d^2/(2*(J_w/r^2+M_w)));
B_withx = [0 0; 1/(den_x*r) 1/(den_x*r) ;0 0; -(M*l*cos(theta)/(r*den))*(1), -(M*l*cos(theta)/(r*den))*(1); 0 0; b*(1) , b*(- 1)];
Q_withx = eye(6)*2;
Q_withx(1,1) = 0.10;
Q_withx(2,2) = 0.10;
R_withx = eye(2)*0.1;
[K_withx,s,e] = lqr(A_withx,B_withx,Q_withx,R_withx,0)

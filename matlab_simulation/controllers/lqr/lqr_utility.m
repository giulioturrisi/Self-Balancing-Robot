[M,com_pos,J_theta,J_phi, M_w, J_w, r,d,g,l] = get_dynamic_parameters_fun();

%linerization around forced eq
state_d = [0 0. 0.5/10 0. 0. 0.];

a = M + 2*M_w + 2*J_w/r^2;

theta=state_d(3);
theta_dot=state_d(4);
phi=state_d(5);
phi_dot=state_d(6);

cos_theta = cos(theta);
sin_theta = sin(theta);
sin_2theta = sin(2*theta);


u_l = (r/2)*(a*M*g*l*sin(theta))/(M*l*cos(theta))
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
Q(2,2) = 0.1
Q(3,3) = 0.2
Q(4,4) = 0.1
R = eye(2)*0.1;
[K,s,e] = lqr(A,B,Q,R,0)


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
Q_withx(1,1) = 1;
Q_withx(2,2) = 1;
Q_withx(5,5) = 0.10;
Q_withx(6,6) = 0.10;
R_withx = eye(2)*0.1;
[K_withx,s,e] = lqr(A_withx,B_withx,Q_withx,R_withx,0)

function [state_new, P_new] = extended_kalman(state_measure,state_old,u_l,u_r,P_old,dt)
%EXTENDED_KALMAN Summary of this function goes here
%   Detailed explanation goes here

V = eye(4)*1;
W = eye(4)*5;

[theta_ddot,phi_ddot,x_ddot] = forward_dynamic_fun(u_l,u_r,state_old);
%integration
state_new_hat = euler_integration_fun(theta_ddot,phi_ddot,x_ddot,state_old,dt);

[A,B] = linearization_discretization_fun(u_l,u_r,state_old(1),state_old(2),state_old(3),state_old(4));
C = eye(4);
P_new_hat = A*P_old*A' + V;


state_new_hat = state_new_hat(3:end);
state_measure = state_measure(3:end);

R = P_new_hat*C'*pinv(C*P_new_hat*C' + W);
v = state_measure' - C*state_new_hat';
state_new = state_new_hat' + R*v;
state_new = state_new';
P_new = P_new_hat - R*C*P_new_hat;


end


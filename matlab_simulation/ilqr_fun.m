function [u_l,u_r] = ilqr_fun(state)
%ILQR Summary of this function goes here
%   Detailed explanation goes here
num_iter_opt = 1;
horizon = 10;
u_l = zeros(1,horizon);
u_r = zeros(1,horizon);

state_vec = zeros(6,horizon);
%state_vec(:,1) = state(3:end)';
state_vec(:,1) = state';

Q = eye(4);
R = eye(2)*100;
P_vec = zeros(4,4*(horizon+1));
P_vec(:,4*(horizon+1) - 3:4*(horizon+1)) = Q;
%P_vec(:,4*(horizon+1) - 3:4*(horizon+1)) = [1.0610    0.0628    0.0000    0.0000;0.0628    0.0657   -0.0000    0.0000;0.0000   -0.0000    6.3822   19.8665;0.0000    0.0000   19.8665  126.7931];
A_vec = [];
B_vec = [];
for iter = 1:num_iter_opt
    
    %backward
    for step = 1:horizon
       %lineariza
       state_actual = state_vec(:,horizon);
       [A_step,B_step] = linearization_discretization_fun(u_l(:,horizon),u_r(:,horizon),state_actual(1),state_actual(2),state_actual(3),state_actual(4));
       A_vec = [A_vec,A_step];
       B_vec = [B_vec,B_step];
       %calculate P_step
       P_next = P_vec(:,4*(horizon+2 - step) - 3:4*(horizon+2-step));
       P_step = Q + A_step'*(P_next - P_next*B_step*pinv(R + B_step'*P_next*B_step)*B_step'*P_next)*A_step;
       P_vec(:,4*(horizon+1 - step) - 3:4*(horizon+1 - step)) = P_step;
    end
    
    %forward
    for step = 1:horizon
       %calculate input
       u_update = (pinv(R+B_vec(:,step*2-1:step*2)'*P_vec(:,step*4 - 3:step*4)*B_vec(:,step*2-1:step*2)))*B_vec(:,step*2-1:step*2)'*P_vec(:,step*4 - 3:step*4)*A_vec(:,step*4-3:step*4)*state_vec(3:end,step);
       u_l(1,step) = u_l(1,step) - u_update(1);
       u_r(1,step) = u_r(1,step) - u_update(2);
       %forward dynamic
       [theta_ddot,phi_ddot,x_ddot] = forward_dynamic_fun(u_l(1,step),u_r(1,step),state_vec(:,step)); 
       %integration
       state_vec(:,step+1) = euler_integration_fun(theta_ddot,phi_ddot,x_ddot,state_vec(:,step),0.01);
    end
    
end

u_l = u_l(1);
u_r = u_r(1);
end


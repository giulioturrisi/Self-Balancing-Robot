function [u_l,u_r] = ilqr_fun(state,state_d,P_f)
%Iterative LQR
state_d = state_d(3:end);

%hyperparameters of the controller
num_iter_opt = 50;
horizon = 500;

u_l = zeros(1,horizon);
u_r = zeros(1,horizon);

%state_vec = zeros(6,horizon);
%state_vec(3:end,:) = zeros(3,horizon) + state_d';
%state_vec(:,1) = state';
state_vec = repmat(state',1,horizon);

Q = eye(4)*1;
Q(1,1) = 5;
R = eye(2)*0.1;
P_vec = zeros(4,4*(horizon+1));
%P_vec(:,4*(horizon+1) - 3:4*(horizon+1)) = 100*Q;
P_vec(:,4*(horizon+1) - 3:4*(horizon+1)) = P_f;
%P_vec(:,4*(horizon+1) - 3:4*(horizon+1)) = [4.4720    0.0048   -0.0000   -0.0000;0.0048    0.0426    0.0000    0.0000; -0.0000    0.0000    7.1157   24.8163;-0.0000    0.0000   24.8163  176.5841];
A_vec = [];
B_vec = [];

V_vec = zeros(4,(horizon+1));
V_vec(:,horizon+1) = P_vec(:,4*(horizon+1) - 3:4*(horizon+1))*(state(3:end)' - state_d');

u_update_v = [0 0]';
u_update_r = [0 0]';
u_update_x = [0 0]';

cost = [0];


for iter = 1:num_iter_opt
P_vec(:,4*(horizon+1) - 3:4*(horizon+1)) = P_f;
V_vec(:,horizon+1) = P_vec(:,4*(horizon+1) - 3:4*(horizon+1))*(state_vec(3:end,horizon) - state_d');
%P_vec(:,4*(horizon+1) - 3:4*(horizon+1)) = [4.4720    0.0048   -0.0000   -0.0000;0.0048    0.0426    0.0000    0.0000; -0.0000    0.0000    7.1157   24.8163;-0.0000    0.0000   24.8163  176.5841];
A_vec = [];
B_vec = [];
u_update_v = [0 0]';
u_update_x = [0 0]';
u_update_r = [0 0]';

    %backward
    for step = 1:horizon-1
       %linearizzo
       state_actual = state_vec(3:end,horizon-step + 1);
       [A_step,B_step] = linearization_discretization_fun(u_l(:,horizon-step),u_r(:,horizon-step),state_actual(1),state_actual(2),state_actual(3),state_actual(4));
       %[A_step,B_step] = linearization_discretization_fun(u_ff,u_ff,state_d(1),state_d(2),state_d(3),state_d(4));
       A_vec = [A_vec,A_step];
       B_vec = [B_vec,B_step];
       
       %calculate P_step
       P_next = P_vec(:,4*(horizon+2 - step) - 3:4*(horizon+2-step));
       
       %P_step = Q + A_step'*(P_next - P_next*B_step*pinv(R + B_step'*P_next*B_step)*B_step'*P_next)*A_step;
       
       Q_uu = R + B_step'*P_next*B_step;
       
       P_step = Q + A_step'*P_next*A_step - (-pinv(Q_uu)*B_step'*P_next*A_step)'*Q_uu*(-pinv(Q_uu)*B_step'*P_next*A_step);
       P_vec(:,4*(horizon+1 - step) - 3:4*(horizon+1 - step)) = P_step;
       
       K = (pinv(R + B_step'*P_next*B_step)*B_step'*P_next*A_step);
       u_vec = [u_l(:,horizon-step+1) u_r(:,horizon-step+1)];
       %disp("calculate as before")
       %v = (A_step - B_step*K)'*V_vec(:,horizon + 2 - step) - K'*R*u_vec' + Q*state_vec(3:end,horizon - step + 1)
       
       Q_u = R*u_vec' + B_step'*V_vec(:,horizon + 2 - step);
       %disp("new")
       v = Q*(state_vec(3:end,horizon - step + 1) - state_d') + A_step'*V_vec(:,horizon + 2 - step) - (-pinv(Q_uu)*B_step'*P_next*A_step)'*Q_uu*(-pinv(Q_uu)*Q_u);
       
       %v = (A_step - B_step*K)'*V_vec(:,horizon + 2 - step);
       eig(A_step - B_step*K);
       V_vec(:,horizon - step + 1) = v;
       %V = (A_step - B_step*K)'*   - K'R*u + Q*state_actual(3,:);
       %V_vec = [V_vec 
    end
    
    
    %forward
    cost_temp = 0;
    state_forward = state_vec;
    for step = 1:horizon-1
       %calculate input
       %error = (state_vec(3:end,step) - state_d');
       error = (state_vec(3:end,step) - state_forward(3:end,step));
       K = (pinv(R+B_vec(:,step*2-1:step*2)'*P_vec(:,step*4 - 3:step*4)*B_vec(:,step*2-1:step*2)))*B_vec(:,step*2-1:step*2)';
       u_update = -K*P_vec(:,step*4 - 3:step*4)*A_vec(:,step*4-3:step*4)*error;
       u_update_x = [u_update_x u_update];
       
       u_v = -K*V_vec(:,step);
       u_update_v = [u_update_v u_v];
       u_vec = [u_l(:,step) u_r(:,step)];
       K_2 = (pinv(R+B_vec(:,step*2-1:step*2)'*P_vec(:,step*4 - 3:step*4)*B_vec(:,step*2-1:step*2)));
       u_2 = -K_2*R*u_vec';
       u_update_r = [u_update_r u_2];

       %u_l(1,step) = u_l(1,step) + u_update(1) + u_v(1)*1 + u_2(1)*1.;
       %u_r(1,step) = u_r(1,step) + u_update(2) + u_v(2)*1 + u_2(2)*1.;
       
       Q_uu = R + B_vec(:,step*2-1:step*2)'*P_vec(:,(step+1)*4 - 3:(step+1)*4)*B_vec(:,step*2-1:step*2);
       Q_ux = B_vec(:,step*2-1:step*2)'*P_vec(:,(step+1)*4 - 3:(step+1)*4)*A_vec(:,step*4-3:step*4);
       Q_u = R*u_vec' + B_vec(:,step*2-1:step*2)'*V_vec(:,step+1);
       
       u_update_k = -pinv(Q_uu)*Q_u;
       u_update_K = -pinv(Q_uu)*Q_ux*(error);
       
       %kK = -R\(R'\[Q_u Q_ux]);
       %u_update_k = kK(:,1);
       %u_update_K = kK(:,2:end)*(error);
       
       
       u_l(1,step) = u_l(1,step) + u_update_k(1)*1 + u_update_K(1);
       u_r(1,step) = u_r(1,step) + u_update_k(2)*1 + u_update_K(2);
       
       
       
       

       %forward dynamic
       [theta_ddot,phi_ddot,x_ddot] = forward_dynamic_fun(u_l(1,step),u_r(1,step),state_vec(:,step)); 
       %integration
       state_vec(:,step+1) = euler_integration_fun(theta_ddot,phi_ddot,x_ddot,state_vec(:,step),0.01);
       
       %cost
       cost_temp = cost_temp + error'*Q*error + [u_l(1,step);u_r(1,step)]'*R*[u_l(1,step);u_r(1,step)];
    end
    cost = [cost cost_temp];
    figure(1);
    plot(state_vec(3,:)); 
    %figure(2);
    %plot(u_l)
    %figure(3);
    %plot(state_vec(5,:)); 
    %figure(4);
    %plot(u_update_v(1,:))
    %figure(5);
    %plot(u_update_x(1,:))
    %figure(6);
    if(abs(cost_temp - cost(iter)) < 0.01)
        break
    end
    %plot(cost)
end

u_l = u_l(1);
u_r = u_r(1);
end


function [u_l,u_r] = lqr_fun(state,state_d,P_f)

horizon = 50;

u_l = zeros(1,horizon);
u_r = zeros(1,horizon);

state_vec = repmat(state',1,horizon);

Q = eye(4)*1;
Q(1,1) = 5;
R = eye(2)*0.1;
P_vec = zeros(4,4*(horizon+1));

P_vec(:,4*(horizon+1) - 3:4*(horizon+1)) = P_f;


P_vec(:,4*(horizon+1) - 3:4*(horizon+1)) = P_f;
A_vec = [];
B_vec = [];
    for step = 1:horizon-1
       %linearizzo
       state_actual = state_vec(3:end,horizon-step + 1);
       [A_step,B_step] = linearization_discretization_fun(u_l(:,horizon-step),u_r(:,horizon-step),state_actual(1),state_actual(2),state_actual(3),state_actual(4));
       A_vec = [A_vec,A_step];
       B_vec = [B_vec,B_step];
       
       %calculate P_step
       P_next = P_vec(:,4*(horizon+2 - step) - 3:4*(horizon+2-step));

       Q_uu = R + B_step'*P_next*B_step;       
       P_step = Q + A_step'*P_next*A_step - (-pinv(Q_uu)*B_step'*P_next*A_step)'*Q_uu*(-pinv(Q_uu)*B_step'*P_next*A_step);
       P_vec(:,4*(horizon+1 - step) - 3:4*(horizon+1 - step)) = P_step;
       
       K = (pinv(R + B_step'*P_next*B_step)*B_step'*P_next*A_step);

    end
    u = -K*(state(3:end)' - state_d(3:end)');
    
    u_l = u(1);
    u_r = u(2);
    
end


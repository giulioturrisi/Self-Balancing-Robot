linear_quadratic_regulator;
%x, x_dot, theta, theta_dot, phi, phi_dot
state = [0 0 1.2 0. 2.0 1];
%control time, initial and final time
dt = 0.01;
t = 0;
t_f = 50;
%eq
%k_lqr = [0.2456    0.7265    0.7071    5.0315; 0.2456    0.7265   -0.7071   -5.0315];
%not eq, but u = 0
%k_lqr = [0.0464    0.7091    0.7071    5.0315; 0.0464    0.7091   -0.7071   -5.0315];
%not eq, u = u_eq
%k_lqr = [0.1822    0.7103   -0.7071   -7.9656; 0.1822    0.7103    0.7071    7.9656];

%k_lqr = [0.0805    0.7105    0.7071    5.0315; 0.0805    0.7105   -0.7071   -5.0315];
cost = [];
total = 0;
state_array = state';
control_input = [0 0]';

state_d = state(3:end);
state_d(1) = -1.0;
state_d(2) = 0.;
state_d(3) = 3;
state_d(4) = 0.;
while t < t_f
    %calculate ILQR 
    [u_l,u_r] = ilqr_fun(state,state_d,P_f,u_ff);
    u_l = u_l + u_ff*0;
    u_r = u_r + u_ff*0;
    
    %calculate LQR
    %u = -k_lqr*(state(3:end)' - state_d');
    %u_l = u(1) + u_ff;
    %u_r = u(2) + u_ff;
    
    control_input = [control_input [u_l u_r]'];

    Q = eye(4);
    R = eye(2);
    cost = [cost,state(3:end)*Q*state(3:end)']; 
    total = total + state(3:end)*Q*state(3:end)';
    
    %forward dynamics
    [theta_ddot,phi_ddot,x_ddot] = forward_dynamic_fun(u_l,u_r,state); 
    %integration
    state = euler_integration_fun(theta_ddot,phi_ddot,x_ddot,state,dt);
    %next step
    t = t + dt;
    
    state_array = [state_array, state'];

     figure(1);
     plot(state_array(3,:))
     figure(2);
     plot(state_array(5,:))
%     figure(3);
%     plot(control_input(1,:))
%     hold on;
%     plot(control_input(2,:))

    
end
% figure(1);
% plot(state_array(3,:))
% figure(2);
% plot(state_array(5,:))
% figure(3);
% plot(control_input(1,:))
% hold on;
% plot(control_input(2,:))
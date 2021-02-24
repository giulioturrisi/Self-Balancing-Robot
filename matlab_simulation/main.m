%here automatically compute LQR gain - change inside the linearization
%point, should be equal to state_d here
linear_quadratic_regulator;

%initial state
%x, x_dot, theta, theta_dot, phi, phi_dot
state = [0 0 0.349 0. .0 0];

%control time, initial and final time
dt = 0.01;
t = 0;
t_f = 50;

%plotting array
cost = [];
x_acc = 0;
total = 0;
state_array = state';
state_array_noise = state(3:end)';
control_input = [0 0]';

%desired state
state_d = state;
state_d(1) = 1.5;
state_d(2) = 0.;
state_d(3) = 0;
state_d(4) = 0.;
state_d(5) = 1.;
state_d(6) = 0.;

%initializationcovariance matric Kalman
P_old = eye(4);

while t < t_f
    %calculate ILQR - the feedforward term (it it exist) is calculated
    %inside the function
    %[u_l,u_r] = ilqr_fun(state,state_d,P_f,u_ff);

    
    %calculate LQR without pos
    %u = -k_lqr*(state(3:end)' - state_d');
    %calculate LQR with pos
    u = -K_withx*(state' - state_d');
    u_l = u(1) + u_ff;
    u_r = u(2) + u_ff;
    
    
    %forward dynamics for state evolution
    [theta_ddot,phi_ddot,x_ddot] = forward_dynamic_fun(u_l,u_r,state);
    %integration
    last_state = state
    state = euler_integration_fun(theta_ddot,phi_ddot,x_ddot,state,dt);
    %add measurement noise
    state_w_noise = state(3:end)' + rand(4,1)*0.02;
    state(3:end) = state_w_noise'
    
    %filter using Kalman
    [state_filtered, P_old] = extended_kalman(state,last_state,u_l,u_r,P_old,dt);
    state(3:end) = state_filtered;
    
    %next step
    t = t + dt;
    
    %for plotting
    x_acc = [x_acc,x_ddot];
    control_input = [control_input [u_l u_r]'];
    state_array = [state_array, state'];
    state_array_noise = [state_array_noise, state_w_noise];
    Q = eye(4);
    R = eye(2);
    cost = [cost,state(3:end)*Q*state(3:end)']; 
    total = total + state(3:end)*Q*state(3:end)';

    
end
 figure();
 plot(state_array(3,:))
 hold on;
 plot(state_array(5,:))
 figure();
 plot(state_array_noise(1,:))
 hold on;
 plot(state_array_noise(3,:))
% figure(3);
% plot(control_input(1,:))
% hold on;
% plot(control_input(2,:))
clear all; close all; clc;

addpath(genpath('./planners'))
addpath(genpath('./estimators'))
addpath(genpath('./integrators'))
addpath(genpath('./controllers'))
addpath(genpath('./controllers/lqr'))
addpath(genpath('./controllers/iterative_lqr'))
addpath(genpath('./controllers/sliding_mode'))


%initial state
%x, x_dot, theta, theta_dot, phi, phi_dot
state = [0 0 0.349 0. .0 0];
state_d = [0 0 0 0 0 0];

%control time, initial and final time
dt = 0.01;
t = 0;
t_f = 10;

%plotting array
x_acc = 0;
total = 0;
state_array = state';
state_array_noise = state(3:end)';
control_input = [0 0]';

%initialization covariance matrix Kalman and final cost
P_kalman = eye(4);
P_f = eye(4);

while t < t_f
    t

    %choose controller
    [u_l,u_r] = lqr_fun(state,state_d,P_f);
    %[u_l,u_r] = ilqr_fun(state,state_d,P_f);
    %[u_l,u_r] = sliding_mode_fun(state,state_d);
    
    
    
    %forward dynamics for state evolution
    [theta_ddot,phi_ddot,x_ddot] = forward_dynamic_fun(u_l,u_r,state);
    
    %integration
    last_state = state;
    state = euler_integration_fun(theta_ddot,phi_ddot,x_ddot,state,dt);
    %add measurement noise
    state_w_noise = state(3:end)' + rand(4,1)*0.0;
    state(3:end) = state_w_noise';
    
    %filter using Kalman
    [state_filtered, P_kalman] = extended_kalman(state,last_state,u_l,u_r,P_kalman,dt);
    state(3:end) = state_filtered;
    
    %next step
    t = t + dt;
    
    %for plotting
    x_acc = [x_acc,x_ddot];
    control_input = [control_input [u_l u_r]'];
    state_array = [state_array, state'];
    state_array_noise = [state_array_noise, state_w_noise];

end
 
figure(1);
plot(state_array(3,:))
hold on;
plot(state_array(5,:))

figure(2);
plot(state_array_noise(1,:))
hold on;
plot(state_array_noise(3,:))

figure(3);
plot(control_input(1,:))
hold on;
plot(control_input(2,:))
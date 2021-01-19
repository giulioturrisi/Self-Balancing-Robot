%x, x_dot, theta, theta_dot, phi, phi_dot
state = [1 0 1 0.1 1. 0.5];
%control time, initial and final time
dt = 0.010;
t = 0;
t_f = 15;
%k_lqr = [0.7087    0.7497    0.7071    4.5129; 0.7087    0.7497   -0.7071   -4.5129]
k_lqr = [0.2456    0.7265    0.7071    4.5129; 0.2456    0.7265   -0.7071   -4.5129]

state_array = state';
while t < t_f
    %calculate ILQR 
    [u_l,u_r] = ilqr_fun(state);
    
    %calculate LQR
    %u = -k_lqr*state(3:end)'
    %u_l = u(1);
    %u_r = u(2);
    
    %forward dynamics
    [theta_ddot,phi_ddot,x_ddot] = forward_dynamic_fun(u_l,u_r,state); 
    %integration
    state = euler_integration_fun(theta_ddot,phi_ddot,x_ddot,state,dt);
    %next step
    t = t + dt;
    
    state_array = [state_array, state'];
    
end

plot(state_array(1,:))

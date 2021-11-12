function new_state = euler_integration_fun(theta_ddot,phi_ddot,x_ddot,state,dt)
%discretization function

x_dot = state(2) + x_ddot*dt;
x = state(1) + x_dot*dt;

theta_dot = state(4) + theta_ddot*dt;
theta = state(3) + theta_dot*dt;

phi_dot = state(6) + phi_ddot*dt;
phi = state(5) + phi_dot*dt;

new_state = [x x_dot theta theta_dot phi phi_dot];
end


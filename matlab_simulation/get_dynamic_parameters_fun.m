function [M, com_pos, J_theta, J_phi, M_w, J_w, r, d, g, l] = get_dynamic_parameters_fun()
%theta_l roation angle wheel left
%theta_r roation angle wheel right
%theta angle pendulum
%M mass chassis
%M_w mass wheel
%phi angle steering
%r radius wheel
%d distance wheel axis - com axis
%l distance com - axis wheel
%J_w inertia wheel
%J_theta inertia chassis along theta
%J_theta inertia chassis along phi

%robot parameters
M = 1.426971; %kg
com_pos = [0 0 10.007]; %mm
M_w = 0.46;
J_theta = 2.167e-02;
J_phi = 4.167e-03;
J_w = 4.968e-03;
r = 0.2;
d = 0.2; %mm
g = 9.8;
l =0.8;
       
end


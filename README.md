Code for a two wheeled inverted pendulum robot using Ros 2. 

- the simulation is performed in Matlab and (separately) in CoppeliaSim using Ros 2.

- the same code used to run the simulation in CoppeliaSim is used to control the real robot, with some additional packages for the electronics (imu, dc motor)

It's possible to control the robot both with the Linear Quadratic Regulator (LQR) or with the Iterative LQR. The state estimation is performed with a Kalman filter or a simple Complementary Filter. 

Still work in progress

import numpy as np
import math

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))
import sys
sys.path.append(dir_path + '/./../')
from robot_model import Robot_Model

import matplotlib.pyplot as plt
import copy

class PID:
    """This is a small class that computes a simple PID control law"""


    def __init__(self, k_x_d, k_pitch, k_pitch_d, k_yaw_d, k_i):
        """
        Args:
            k_x_d (float): linear velocity gain
            k_pitch (float): pitch angular position gain
            k_pitch_d (float): pitch angular velocity gain
            k_yaw_d (float): yaw angular velocity gain
            k_i (float): pitch angular position integral gain, maybe better on the velocity!!
        """
        self.k_i = k_i 
        self.k_x_d = k_x_d
        self.k_pitch = k_pitch
        self.k_pitch_d = k_pitch_d
        self.k_yaw_d = k_yaw_d

        self.integral_e_pitch = 0

        self.state_dim = 6
        self.state_integral_dim = 2
        self.control_dim = 2

        self.twip = Robot_Model()



    def calculate_errors(self, state, state_des):
        """Compute PID errors signal
        Args:
            state (np.array): actual state of the robot
            state_des (np.array): desired state

        Returns:
            (np.array): linear and angular error velocity 
        """
        e_pitch = self.k_pitch*(state_des[1] - state[1])  
        e_pitch_d =  self.k_pitch_d*(state_des[4] - state[4])
        self.integral_e_pitch += e_pitch
        e_pitch_i = self.k_i*(self.integral_e_pitch)

        e_vel = self.k_x_d*(state_des[3] - state[3])  
        
        e_yaw_d = self.k_yaw_d*(state_des[5] - state[5])

        return np.array((e_vel, e_pitch + e_pitch_d + e_pitch_i, e_yaw_d))
        


    def compute_control(self, state, state_des):
        """Compute PID control inputs
        Args:
            state (np.array): actual robot state
            state_des (np.array): desired robot state

        Returns:
            (np.array): control inputs
        """
        gen_forces = self.calculate_errors(state, state_des)

        control_matrix = self.twip.inv_control_matrix()

        u_ff = self.twip.compute_feed_forward(state_des[1], state_des[3])

        torque = control_matrix@gen_forces

        return [u_ff + torque.item(0),u_ff + torque.item(1)]



if __name__=="__main__":

    dt = 0.005
    k_x_d = 0        
    k_pitch = 500
    k_pitch_d = 50
    k_yaw_d = 0.5
    k_i = 0.001
    controller = PID(k_x_d = k_x_d, k_pitch = k_pitch, k_pitch_d = k_pitch_d, 
                    k_yaw_d = k_yaw_d, k_i = k_i)



    state = np.array([0, 0.1, 0, 0.4, 0.7, 0])
    state_des = np.array([0, 0, 0, 0, 0., 0.])
    max_input = 0.5
    state_evolution = [copy.copy(state)]

    robot = Robot_Model()

    for j in range(0, 2000):
        control = controller.compute_control(state, state_des)

        tau_l = np.where(control[0] > max_input, max_input, control[0])
        tau_l = np.where(control[0] < -max_input, -max_input, control[0])
        
        tau_r = np.where(control[1] > max_input, max_input, control[1])
        tau_r = np.where(control[1] < -max_input, -max_input, control[1])

        tau = np.array([tau_l, tau_r]).reshape(controller.control_dim,)

        
        qdd = robot.forward_dynamics(state.reshape(controller.state_dim,), tau)
        qdd = qdd[3:6]
        state = robot.euler_integration(state, qdd, dt).reshape(controller.state_dim,)
        state_evolution = np.append(state_evolution, [copy.copy(state)], axis=0)
            

    # Plotting ---------------------------------------
    fig, axs = plt.subplots(2, 2)
    fig.set_figheight(8)
    fig.set_figwidth(10)
    # Setting the values for all axes.
    custom_xlim = (0, 2000)
    custom_ylim = (-2, 2)
    plt.setp(axs, xlim=custom_xlim, ylim=custom_ylim)
    # Data to Plot
    axs[0, 0].plot(state_evolution[:,1])
    axs[0, 0].set_title('pitch')
    axs[0, 1].plot(state_evolution[:,4])
    axs[0, 1].set_title('pitch_d')
    axs[1, 0].plot(state_evolution[:,3])
    axs[1, 0].set_title('x_d')
    axs[1, 1].plot(state_evolution[:,5])
    axs[1, 1].set_title('yaw_d')
    plt.show()


import numpy as np
import math

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))
import sys
sys.path.append(dir_path + '/./../')
from robot_model import Robot_Model

import matplotlib.pyplot as plt
import copy

class Sliding_Mode:
    """This is a small class that computes a Sliding Mode control law - very brutal!"""


    def __init__(self, k_s, k_x_d, k_pitch, k_pitch_d, k_yaw_d):
        """
        Args:
            k_s (float): sliding surface gain
            k_x_d (float): linear velocity gain
            k_pitch (float): pitch angular position gain
            k_pitch_d (float): pitch angular velocity gain
            k_yaw_d (float): yaw angular velocity gain
        """
        self.k_s = k_s 
        self.k_x_d = k_x_d
        self.k_pitch = k_pitch
        self.k_pitch_d = k_pitch_d
        self.k_yaw_d = k_yaw_d

        self.state_dim = 6
        self.control_dim = 2

        self.twip = Robot_Model()



    def calculate_sliding_surface(self,state, state_des):
        """Compute sliding surface
        Args:
            state (np.array): actual state of the robot
            state_des (np.array): desired state
        Returns:
            (np.array): sliding surface for pitch, yaw, and linear velocity 
        """
        s2 = self.k_pitch*(state_des[1] - state[1])  + self.k_pitch_d*(state_des[4] - state[4])
        s1 = self.k_x_d*(state_des[3] - state[3])  
        s3 = self.k_yaw_d*(state_des[5] - state[5])

        return np.array((s1, s2, s3))



    def compute_control(self, state, state_des):
        """Compute SLiding Mode control inputs
        Args:
            state (np.array): actual robot state
            state_des (np.array): desired robot state
        Returns:
            (np.array): control inputs
        """

        state_des[1] = self.twip.compute_angle_from_vel(state_des[3])
        u_ff = self.twip.compute_feed_forward(state_des[1], state_des[3])

        s = self.calculate_sliding_surface(state, state_des)
        
        # TO DO: calculate equivalent control ------------------------------
        
        #gen_forces = -self.k_s*np.sign(s)
        gen_forces = self.k_s*np.tanh(s) 
        control_matrix = self.twip.inv_control_matrix()
        
        torque = control_matrix@gen_forces

        return [u_ff + torque.item(0),u_ff + torque.item(1)]



if __name__=="__main__":

    dt = 0.005
    k_s = 100
    k_x_dot = 0.0
    k_pitch = 100
    k_pitch_dot = 15
    k_yaw_dot = 0.3
    controller = Sliding_Mode(k_s, k_x_dot, k_pitch, k_pitch_dot, k_yaw_dot)



    state = np.array([0, 0.1, 0, 0.4, 0.7, 0])
    state_des = np.array([0, 0, 0, 0, 0., 0.])
    state_evolution = [copy.copy(state)]

    robot = Robot_Model()

    for j in range(0, 2000):
        control = controller.compute_control(state, state_des)
        tau = np.array([control[0], control[1]]).reshape(controller.control_dim,)

        
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


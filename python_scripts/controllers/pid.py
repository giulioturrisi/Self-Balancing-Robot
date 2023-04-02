import numpy as np
import math

import sys
sys.path.append('/home/python_scripts/')
from robot_model import Robot_Model



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

        self.twip = Robot_Model()



    def calculate_errors(self,state, state_des):
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

        u_ff = self.twip.compute_feed_forward(state_des[1])

        torque = control_matrix@gen_forces

        return [u_ff + torque.item(0),u_ff + torque.item(1)]

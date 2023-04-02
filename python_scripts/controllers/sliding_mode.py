import numpy as np
import math

import sys
sys.path.append('/home/python_scripts/')
from robot_model import Robot_Model

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
        print("angle des", state_des[1])
        u_ff = self.twip.compute_feed_forward(state_des[1], state_des[3])

        s = self.calculate_sliding_surface(state, state_des)
        
        # TO DO: calculate equivalent control ------------------------------
        
        #gen_forces = -self.k_s*np.sign(s)
        gen_forces = self.k_s*np.tanh(s) 
        control_matrix = self.twip.inv_control_matrix()
        
        torque = control_matrix@gen_forces

        return [u_ff + torque.item(0),u_ff + torque.item(1)]


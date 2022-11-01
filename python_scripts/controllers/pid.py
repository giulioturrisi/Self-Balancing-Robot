import numpy as np
import math

import sys
sys.path.append('/home/python_scripts/')
import forward_dynamics

class PID:
    def __init__(self, k_x_d, k_pitch, k_pitch_d, k_yaw_d, k_i):
        self.k_i = k_i 
        self.k_x_d = k_x_d
        self.k_pitch = k_pitch
        self.k_pitch_d = k_pitch_d
        self.k_yaw_d = k_yaw_d

        self.integral_e_pitch = 0

    def calculate_errors(self,state, state_des):
        e_pitch = self.k_pitch*(state_des[1] - state[1])  + self.k_pitch_d*(state_des[4] - state[4])
        e_vel = self.k_x_d*(state_des[3] - state[3])  
        e_yaw = self.k_yaw_d*(state_des[5] - state[5])

        e_pitch = e_pitch + self.k_i*(self.integral_e_pitch)
        self.integral_e_pitch = e_pitch

        return np.array((e_vel, e_pitch, e_yaw))

    def compute_control(self, state, state_des):
        gen_forces = self.calculate_errors(state, state_des)

        control_matrix = forward_dynamics.inv_control_matrix()
        print("control matrix", control_matrix)
        print("generilized forces", gen_forces)
        print("mapping to torque", control_matrix@gen_forces)
        torque = -control_matrix@gen_forces
        print("torque_l: ", torque.item(0))
        print("torque_r: ", torque.item(1))
        return [torque.item(0),torque.item(1)]





#if __name__=="__main__":

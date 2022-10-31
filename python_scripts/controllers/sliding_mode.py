import numpy as np
import math

import sys
sys.path.append('/home/python_scripts/')
import forward_dynamics

class Sliding_Mode:
    def __init__(self, k_s, k_x_d, k_roll, k_roll_d, k_yaw_d):
        self.k_s = k_s 
        self.k_x_d = k_x_d
        self.k_roll = k_roll
        self.k_roll_d = k_roll_d
        self.k_yaw_d = k_yaw_d

    def calculate_sliding_surface(self,state, state_des):
        s2 = self.k_roll*(state_des[1] - state[1])  + self.k_roll_d*(state_des[4] - state[4])
        s1 = self.k_x_d*(state_des[3] - state[3])  
        s3 = self.k_yaw_d*(state_des[5] - state[5])

        print("s1",s1)
        print("s2",s2)

        return np.array((s1, s2, s3))

    def compute_control(self, state, state_des):
        s = self.calculate_sliding_surface(state, state_des)
        #calculate equivalent control
        #gen_forces = -self.k_s*np.sign(s)
        gen_forces = -self.k_s*np.tanh(s) 
        control_matrix = forward_dynamics.inv_control_matrix()
        print("control matrix", control_matrix)
        print("generilized forces", gen_forces)
        print("mapping to torque", control_matrix@s)
        torque = control_matrix@gen_forces
        print("torque_l: ", torque.item(0))
        print("torque_r: ", torque.item(1))
        return [torque.item(0),torque.item(1)]





#if __name__=="__main__":

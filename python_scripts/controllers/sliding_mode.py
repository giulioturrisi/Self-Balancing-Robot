import numpy as np
import math

import sys
sys.path.append('/home/python_scripts/')
from twip_dynamics import Twip_dynamics

class Sliding_Mode:
    def __init__(self, k_s, k_x_d, k_pitch, k_pitch_d, k_yaw_d):
        self.k_s = k_s 
        self.k_x_d = k_x_d
        self.k_pitch = k_pitch
        self.k_pitch_d = k_pitch_d
        self.k_yaw_d = k_yaw_d

        self.twip = Twip_dynamics()

    def calculate_sliding_surface(self,state, state_des):
        s2 = self.k_pitch*(state_des[1] - state[1])  + self.k_pitch_d*(state_des[4] - state[4])
        s1 = self.k_x_d*(state_des[3] - state[3])  
        s3 = self.k_yaw_d*(state_des[5] - state[5])

        print("s1",s1)
        print("s2",s2)

        return np.array((s1, s2, s3))

    def compute_control(self, state, state_des):

        state_des[1] = self.twip.compute_angle_from_vel(state_des[3])
        print("angle des", state_des[1])
        u_ff = self.twip.compute_feed_forward(state_des[1], state_des[3])

        s = self.calculate_sliding_surface(state, state_des)
        #calculate equivalent control
        #gen_forces = -self.k_s*np.sign(s)
        gen_forces = self.k_s*np.tanh(s) 
        control_matrix = self.twip.inv_control_matrix()

        
        torque = control_matrix@gen_forces

        return [u_ff + torque.item(0),u_ff + torque.item(1)]





#if __name__=="__main__":

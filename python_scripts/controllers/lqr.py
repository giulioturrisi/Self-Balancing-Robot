import control
import numpy as np

import sys
sys.path.append('/home/python_scripts/')
import forward_dynamics

class LQR:
    def __init__(self):
        self.lin_state = np.zeros(6)
        self.lin_tau = np.zeros(2)

        self.Q = np.identity(6)
        
        self.Q[0,0] = 0.0 #x
        self.Q[2,2] = 0.0 #yaw

        self.R = np.identity(2)*10
        
        self.K = self.calculate_LQR_gain(self.lin_state, self.lin_tau)

    def calculate_LQR_gain(self,lin_state, lin_tau):
        A = forward_dynamics.compute_A_matrix(lin_state, lin_tau)
        B = forward_dynamics.compute_B_matrix(lin_state, lin_tau)

        print("A", A)
        print("B", B)

        K, S, E = control.lqr(A, B, self.Q, self.R)

        print("K",K)

        return K

    def compute_control(self, state, state_des):
        print("state", state)
        print("state des", state_des)
        return self.K@(state-state_des)





if __name__=="__main__":
    LQR()

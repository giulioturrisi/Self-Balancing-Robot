import control
import numpy as np

import sys
sys.path.append('/home/python_scripts/')
import forward_dynamics

class LQR:
    def __init__(self, lin_state = None, lin_tau = None, horizon = None, dt = None):
        self.lin_state = np.zeros(6)
        self.lin_tau = np.zeros(2)
        self.horizon = 2000
        self.dt = 0.001

        self.Q = np.identity(6)
        
        self.Q[0,0] = 0.0 #x
        self.Q[3,3] = 0 #x_d
        self.Q[2,2] = 0.0 #yaw
        
        self.Q[1,1] = 1.0 #pitch
        self.Q[5,5] = 0.2 #pitch

        self.R = np.identity(2)*10
        
        #self.K = self.calculate_continuous_LQR_gain(self.lin_state, self.lin_tau)
        self.K = self.calculate_discrete_LQR_gain(self.lin_state, self.lin_tau)

    def calculate_continuous_LQR_gain(self,lin_state, lin_tau):
        A = forward_dynamics.compute_A_matrix(lin_state, lin_tau)
        B = forward_dynamics.compute_B_matrix(lin_state, lin_tau)

        K, S, E = control.lqr(A, B, self.Q, self.R)

        print("K continuous",K)

        return K

    def calculate_discrete_LQR_gain(self,lin_state, lin_tau):

        P_next = np.identity(6)
        P_next[0,0] = 0.0 #x
        P_next[2,2] = 0.0 #yaw

        A = forward_dynamics.compute_A_matrix(lin_state, lin_tau)
        B = forward_dynamics.compute_B_matrix(lin_state, lin_tau)

        A_discrete = A*self.dt + np.identity(6)
        B_discrete = B*self.dt


        for i in range(0, self.horizon):
            Q_uu = self.R + B_discrete.T@P_next@B_discrete

            temp = (-np.linalg.pinv(Q_uu)@B_discrete.T@P_next@A_discrete)
            P_next = self.Q + A_discrete.T@P_next@A_discrete - temp.T@Q_uu@temp
            
            self.K = (np.linalg.pinv(self.R + B_discrete.T@P_next@B_discrete)@B_discrete.T@P_next@A_discrete)

        print("K discrete", self.K)
        return self.K

    def compute_control(self, state, state_des):
        u_ff = forward_dynamics.compute_feed_forward(state_des[1])
        u_ff = np.ones(2)*u_ff

        #self.K = self.calculate_discrete_LQR_gain(state_des, u_ff)
        
        print("u_ff", u_ff)
        print("state des", state_des)
        return u_ff + self.K@(state_des - state)





if __name__=="__main__":
    LQR()

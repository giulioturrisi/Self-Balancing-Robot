import numpy as np

import sys
sys.path.append('/home/python_scripts/')
from twip_dynamics import Twip_dynamics

class LQI:
    def __init__(self, lin_state = None, lin_tau = None, horizon = None, dt = None):
        self.lin_state = lin_state
        self.lin_tau = np.zeros(2)
        self.horizon = 2000
        self.dt = dt

        self.twip = Twip_dynamics()

        self.reference_x_d = lin_state[3]
        self.reference_yaw_d = lin_state[5]

        self.integral_error_x_d = 0
        self.integral_error_yaw_d = 0

        self.state_dim = 6
        self.state_integral_dim = 2
        self.control_dim = 2

        self.Q = np.identity(self.state_dim + self.state_integral_dim)
        
        self.Q[0,0] = 0.0 #x
        self.Q[1,1] = 5.0 #pitch
        self.Q[2,2] = 0.0 #yaw
        
        self.Q[3,3] = 2 #x_d
        self.Q[4,4] = 1.0 #pitch_d
        self.Q[5,5] = 0.2 #yawd
        self.Q[6,6] = 2 #integral_error_x_d
        self.Q[7,7] = 0.2 #integral_error_yaw_d

        self.R = np.identity(self.control_dim)*10
        
 
        self.K = self.calculate_discrete_LQR_gain(self.lin_state, self.lin_tau)



    def calculate_discrete_LQR_gain(self,lin_state, lin_tau):

        P_next = np.identity(self.state_dim + self.state_integral_dim)
        P_next[0,0] = 0.0 #x
        P_next[2,2] = 0.0 #yaw

        A = self.twip.A_f_integral(lin_state, lin_tau)
        B = self.twip.B_f_integral(lin_state, lin_tau)

        A_discrete = A*self.dt + np.identity(self.state_dim + self.state_integral_dim)
        B_discrete = B*self.dt


        for i in range(0, self.horizon):
            Q_uu = self.R + B_discrete.T@P_next@B_discrete

            temp = (-np.linalg.pinv(Q_uu)@B_discrete.T@P_next@A_discrete)
            P_next = self.Q + A_discrete.T@P_next@A_discrete - temp.T@Q_uu@temp
            
            self.K = (np.linalg.pinv(self.R + B_discrete.T@P_next@B_discrete)@B_discrete.T@P_next@A_discrete)

        print("K discrete", self.K)
        return self.K

    def compute_control(self, state, state_des):
        state_des[1] = self.twip.compute_angle_from_vel(state_des[3])
        u_ff = self.twip.compute_feed_forward(state_des[1], state_des[3])
        u_ff = np.ones(2)*u_ff

        self.integral_error_x_d = self.integral_error_x_d + (state_des[3] - state[3])*self.dt*10.
        self.integral_error_yaw_d = self.integral_error_yaw_d + (state_des[5] - state[5])*self.dt*10.

        print("integral_error_x_d", self.integral_error_x_d)
        print("integral_error_yaw_d", self.integral_error_yaw_d)
        
        #errors = np.hstack((state_des - state, self.integral_error_x_d))
        errors = np.hstack((state_des - state, self.integral_error_x_d, self.integral_error_yaw_d))

        #print("errors", errors)
        
        return u_ff + self.K@(errors) 





if __name__=="__main__":
    LQI()

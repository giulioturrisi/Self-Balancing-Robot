import control
import numpy as np

import sys
sys.path.append('/home/python_scripts/')
import forward_dynamics

class LQI:
    def __init__(self, lin_state = None, lin_tau = None, horizon = None, dt = None):
        self.lin_state = lin_state
        self.lin_tau = np.zeros(2)
        self.horizon = 2000
        self.dt = dt

        self.reference_vel = lin_state[3]

        self.integral_error_vel = 0

        self.Q = np.identity(7)
        
        self.Q[0,0] = 0.0 #x
        self.Q[1,1] = 5.0 #pitch
        self.Q[2,2] = 0.0 #yaw
        
        self.Q[3,3] = 2 #x_d
        self.Q[4,4] = 1.0 #pitch_d
        self.Q[5,5] = 0.2 #yawd
        self.Q[6,6] = 2 #yawd

        self.R = np.identity(2)*10
        
 
        self.K = self.calculate_discrete_LQR_gain(self.lin_state, self.lin_tau)



    def calculate_discrete_LQR_gain(self,lin_state, lin_tau):

        P_next = np.identity(7)
        P_next[0,0] = 0.0 #x
        P_next[2,2] = 0.0 #yaw

        A = forward_dynamics.compute_A_matrix_integral(lin_state, self.reference_vel, lin_tau)
        B = forward_dynamics.compute_B_matrix_integral(lin_state, self.reference_vel, lin_tau)

        A_discrete = A*self.dt + np.identity(7)
        B_discrete = B*self.dt


        for i in range(0, self.horizon):
            Q_uu = self.R + B_discrete.T@P_next@B_discrete

            temp = (-np.linalg.pinv(Q_uu)@B_discrete.T@P_next@A_discrete)
            P_next = self.Q + A_discrete.T@P_next@A_discrete - temp.T@Q_uu@temp
            
            self.K = (np.linalg.pinv(self.R + B_discrete.T@P_next@B_discrete)@B_discrete.T@P_next@A_discrete)

        print("K discrete", self.K)
        return self.K

    def compute_control(self, state, state_des):
        state_des[1] = forward_dynamics.compute_angle_from_vel(state_des[3])
        u_ff = forward_dynamics.compute_feed_forward(state_des[1], state_des[3])
        u_ff = np.ones(2)*u_ff

        self.integral_error_vel = self.integral_error_vel + (state_des[3] - state[3])*self.dt*10.
        print("feed forward", u_ff)
        print("angle to reach", state_des[1])
        print("error angle des", state_des[1] - state[1])
        print("error vel des", state_des[3] - state[3])
        print("integral error", self.integral_error_vel)
        #print("self.integral_error_vel", self.integral_error_vel)
        errors = np.hstack((state_des - state, self.integral_error_vel))
        #print("errors", errors)
        return u_ff + self.K@(errors) 





if __name__=="__main__":
    LQI()

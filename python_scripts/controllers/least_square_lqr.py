import numpy as np

import sys
sys.path.append('/home/python_scripts/')
import euler_integration
from twip_dynamics import Twip_dynamics

class LQR:
    def __init__(self, lin_state = None, lin_tau = None, horizon = None, dt = None):
        self.lin_state = np.zeros(6)
        self.lin_tau = np.zeros(2)
        self.horizon = 2000
        self.dt = dt

        self.twip = Twip_dynamics()

        self.Q = np.identity(6)
        
        self.Q[0,0] = 0.0 #x
        self.Q[3,3] = 2 #x_d
        self.Q[2,2] = 0.0 #yaw
        
        self.Q[1,1] = 5.0 #pitch
        self.Q[4,4] = 1.0 #pitch_d
        self.Q[5,5] = 0.2 #yawd

        self.R = np.identity(2)*10
        
        #self.K = self.calculate_continuous_LQR_gain(self.lin_state, self.lin_tau)
        self.K = self.calculate_discrete_LQR_gain(self.lin_state, self.lin_tau)

        self.phi_vec = np.array([])
        self.error_vec = np.array([])
        self.best_param = None


    def calculate_discrete_LQR_gain(self,lin_state, lin_tau):

        P_next = np.identity(6)
        P_next[0,0] = 0.0 #x
        P_next[2,2] = 0.0 #yaw

        A = self.twip.A_f(lin_state, lin_tau)
        B = self.twip.B_f(lin_state, lin_tau)

        A_discrete = A*self.dt + np.identity(6)
        B_discrete = B*self.dt


        for i in range(0, self.horizon):
            Q_uu = self.R + B_discrete.T@P_next@B_discrete

            temp = (-np.linalg.pinv(Q_uu)@B_discrete.T@P_next@A_discrete)
            P_next = self.Q + A_discrete.T@P_next@A_discrete - temp.T@Q_uu@temp
            
            self.K = (np.linalg.pinv(self.R + B_discrete.T@P_next@B_discrete)@B_discrete.T@P_next@A_discrete)

        print("K discrete", self.K)
        return self.K

    def lift_space(self, state):
        return np.array([1, state[1]*state[1], state[2]*state[2], state[3]*state[3], state[4]*state[4], state[5]*state[5]])


    def update_dataset(self, previous_state, control, state_meas):
        next_state = self.twip.forward_dynamics(previous_state, control)
        qdd = next_state[3:6]
        state_pred = euler_integration.euler_integration(previous_state, qdd, self.dt)
        
        # should be 1 x num_features
        state_pred_lift = self.lift_space(state_pred)
        state_pred_lift = state_pred_lift.reshape(1,6)

        # should be n_datapoints x num_features
        if(not np.any(self.phi_vec)):
            self.phi_vec = state_pred_lift
        else:
            self.phi_vec = np.append(self.phi_vec, state_pred_lift, axis=0)

        # should be n_datapoints x num_features
        error = state_meas - state_pred[1:]
        error = error.reshape(1,5)
        if(not np.any(self.error_vec)):
            self.error_vec = error
        else:
            self.error_vec = np.append(self.error_vec, error, axis=0)
        
    def update_parameters(self,):
        # least square
        self.best_param = np.linalg.pinv(self.phi_vec.T@self.phi_vec)@self.phi_vec.T@self.error_vec

        print("best_param", self.best_param)


        

    def compute_control(self, state, state_des):
        state_des[1] = self.twip.compute_angle_from_vel(state_des[3])
        u_ff = self.twip.compute_feed_forward(state_des[1], state_des[3])
        u_ff = np.ones(2)*u_ff

        #self.K = self.calculate_discrete_LQR_gain(state_des, u_ff)
        

        return u_ff + self.K@(state_des - state)




if __name__=="__main__":
    control = LQR(dt=0.01)
    
    x = np.array([0, 0, 0, 0, 1., 0.])
    u = np.array([0.1, 0.1])
    y_meas = np.array([0, 0, 0, 0.95, 0])

    control.update_dataset(x, u, y_meas)

    x2 = np.array([0, 0, 0, 0, 0.95, 0.])
    u2 = np.array([0.1, 0.1])
    y2_meas = np.array([0, 0, 0, 1.9, 0])
    control.update_dataset(x2, u2, y2_meas)

    control.update_parameters()
    
    

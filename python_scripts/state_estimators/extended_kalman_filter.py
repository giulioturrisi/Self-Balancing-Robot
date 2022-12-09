import numpy as np

import sys
sys.path.append('/home/python_scripts/')
import euler_integration
from twip_dynamics import Twip_dynamics

class Extended_Kalman:
    def __init__(self,):
        print("init EKF")

        self.state_dim = 6
        self.control_dim = 2

        # process noise
        self.V = np.eye(self.state_dim)*0.1
        # measurement noise
        self.W = np.eye(self.state_dim)*50

        # covariance initialization
        self.P = np.eye(self.state_dim)

        # observation matrix
        self.C = np.eye(self.state_dim)

    def compute_prediction(self, state, control):
        qdd = self.twip.fd(state,control)
        qdd = qdd[3:6]
        # integration
        predicted_state = euler_integration.euler_integration(state, qdd, self.dt).reshape(self.state_dim,1)

        A_step = self.twip.A_f(state, control)
        A_step = np.array(A_step)
        B_step = self.twip.B_f(state, control)
        B_step = np.array(B_step)

        A_step = A_step*self.dt + np.identity(self.state_dim)
        B_step = B_step*self.dt

        self.P = A_step@self.P@A_step.T + self.V;

        return predicted_state


    def compute_correction(self, predicted_state, measurements):
        R = self.P@self.C.T@np.linalg.pinv(self.C@self.P@self.C.T + self.W)
        v = measurements - predicted_state
        state_new = predicted_state.T + self.R*v

        self.P = self.P - self.R@self.C@self.P

        return state_new


    def filter(self, state, control, measurements):
        predicted_state = self.compute_prediction(state, control)
        return self.compute_correction(predicted_state, measurements)

    
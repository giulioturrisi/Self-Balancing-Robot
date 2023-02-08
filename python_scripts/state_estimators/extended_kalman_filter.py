import numpy as np

import sys
sys.path.append('/home/python_scripts/')
import euler_integration
from twip_dynamics import Twip_dynamics



class Extended_Kalman_Filter:
    """This is a small class that computes an EKF for state estimation"""


    def __init__(self, V, W, P):
        """
        Args:
            V (float): process noise
            W (float): measurement noise
            P (float); initial covariance
        """

        self.state_dim = 6
        self.control_dim = 2

        # process noise
        self.V = np.eye(self.state_dim)*V
        # measurement noise
        self.W = np.eye(self.state_dim)*W

        # covariance initialization
        self.P = np.eye(self.state_dim)*P

        # observation matrix
        self.C = np.eye(self.state_dim)



    def compute_prediction(self, state, control):
        """Compute state prediction

        Args:
            state (np.array): state of the robot at time K-1
            control (np.array): control applied to the robot at time K-1

        Returns:
            (np.array): predicted state at time K
        """
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
        """Compute state correction

        Args:
            predicted_state (np.array): predicted state at time K
            measurements (np.array): measured state at time K

        Returns:
            (np.array): filtered state at time K
        """
        R = self.P@self.C.T@np.linalg.pinv(self.C@self.P@self.C.T + self.W)
        v = measurements - predicted_state
        state_new = predicted_state.T + self.R*v

        self.P = self.P - self.R@self.C@self.P

        return state_new



    def compute_filter(self, state, control, measurements):
        """Compute prediction and correction step of the EKF filter

        Args:
            state (np.array): state of the robot at time K-1
            control (np.array): control applied to the robot at time K-1
            measurements (np.array): measured state at time K

        Returns:
            (np.array): filtered state at time K
        """
        predicted_state = self.compute_prediction(state, control)
        return self.compute_correction(predicted_state, measurements)

    
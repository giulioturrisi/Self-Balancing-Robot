import numpy as np

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))
import sys
sys.path.append(dir_path + '/./../')
from robot_model import Robot_Model

import matplotlib.pyplot as plt
import copy

class LQI:
    """This is a small class that computes an LQR control law using integral actions
       for the yaw and pitch angular velocities"""


    def __init__(self, lin_state = None, lin_tau = None, horizon = None, dt = None):
        """
        Args:
            lin_state (np.array): linearization state
            lin_tau (np.array): linearization control inputs
            horizon (int): how much to look into the future for optimizing the gains 
            dt (int): desidered sampling time
        """
        self.lin_state = lin_state
        self.lin_tau = np.zeros(2)
        self.horizon = 2000
        self.dt = dt

        self.twip = Robot_Model()

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
        """Calculate by backward iterations the optimal LQR gains
        Args:
            lin_state (np.array): linearization state
            lin_tau (np.array): linearization control inputs
        Returns:
             K (np.array): optimal gains
        """
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

        return self.K



    def compute_control(self, state, state_des):
        """Compute feedforward and LQR control inputs
        Args:
            state (np.array): actual robot state
            state_des (np.array): desired robot state

        Returns:
            (np.array): optimized control inputs
        """
        state_des[1] = self.twip.compute_angle_from_vel(state_des[3])
        u_ff = self.twip.compute_feed_forward(state_des[1], state_des[3])
        u_ff = np.ones(2)*u_ff

        self.integral_error_x_d = self.integral_error_x_d + (state_des[3] - state[3])*self.dt*10.
        self.integral_error_yaw_d = self.integral_error_yaw_d + (state_des[5] - state[5])*self.dt*10.

        print("integral_error_x_d", self.integral_error_x_d)
        print("integral_error_yaw_d", self.integral_error_yaw_d)
        
        #errors = np.hstack((state_des - state, self.integral_error_x_d))
        errors = np.hstack((state_des - state, self.integral_error_x_d, self.integral_error_yaw_d))

        
        return u_ff + self.K@(errors) 


if __name__=="__main__":
    
    dt = 0.01
    lin_state = np.zeros(8)
    
    controller = LQI(dt=dt, lin_state=lin_state)

    state = np.array([0, 0.1, 0, 0.4, 0.7, 0])
    state_des = np.array([0, 0, 0, 0, 0., 0.])
    state_evolution = [copy.copy(state)]

    robot = Robot_Model()

    for j in range(0, 2000):
        control = controller.compute_control(state, state_des)
        tau = np.array([control[0], control[1]]).reshape(controller.control_dim,)

        
        qdd = robot.forward_dynamics(state.reshape(controller.state_dim,), tau)
        qdd = qdd[3:6]
        state = robot.euler_integration(state, qdd, dt).reshape(controller.state_dim,)
        state_evolution = np.append(state_evolution, [copy.copy(state)], axis=0)
            

    # Plotting ---------------------------------------
    fig, axs = plt.subplots(2, 2)
    fig.set_figheight(8)
    fig.set_figwidth(10)
    # Setting the values for all axes.
    custom_xlim = (0, 100)
    custom_ylim = (-2, 2)
    plt.setp(axs, xlim=custom_xlim, ylim=custom_ylim)
    # Data to Plot
    axs[0, 0].plot(state_evolution[:,1])
    axs[0, 0].set_title('pitch')
    axs[0, 1].plot(state_evolution[:,4])
    axs[0, 1].set_title('pitch_d')
    axs[1, 0].plot(state_evolution[:,3])
    axs[1, 0].set_title('x_d')
    axs[1, 1].plot(state_evolution[:,5])
    axs[1, 1].set_title('yaw_d')
    plt.show()


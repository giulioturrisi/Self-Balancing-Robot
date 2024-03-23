import numpy as np

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))
import sys
sys.path.append(dir_path + '/./../')
from robot_model import Robot_Model

import casadi as cs
import matplotlib.pyplot as plt
import copy

class Adaptive_LQR:
    """This is a small class that computes an LQR control law based on a least square update"""


    def __init__(self, lin_state = None, lin_tau = None, horizon = None, dt = None):
        """
        Args:
            lin_state (np.array): linearization state
            lin_tau (np.array): linearization control inputs
            horizon (int): how much to look into the future for optimizing the gains 
            dt (int): desidered sampling time
        """
        self.lin_state = np.zeros(6)
        self.lin_tau = np.zeros(2)
        if(horizon==None):
            self.horizon = 2000
        else:
            self.horizon = horizon
        self.dt = dt

        self.twip = Robot_Model()
        self.state_dim = 6
        self.control_dim = 2
        

        self.Q = np.zeros((6, 6))
        self.Q[0,0] = 0.0 #x
        self.Q[3,3] = 2 #x_d
        self.Q[2,2] = 0.0 #yaw
        self.Q[1,1] = 5.0 #pitch
        self.Q[4,4] = 1.0 #pitch_d
        self.Q[5,5] = 0.2 #yawd
        self.R = np.identity(2)*10

        self.P_next = np.identity(6)


        self.error_vec = np.array([])

        self.K = self.calculate_discrete_LQR_gain(self.lin_state, self.lin_tau, self.horizon)

        # adaptive law parameters -----------------------------------------------------
        self.gamma = 0.005
        self.adaptive_gains = np.array([[0, 0., 0, 0]])


        # calculate error gradient for MIT rule ---------------------------------------
        previous_state = cs.SX.sym("previous_state", 6, 1)
        state_des = cs.SX.sym("state_des", 6, 1)
        state_meas = cs.SX.sym("state_meas", 6, 1)
        adaptive_gains = cs.SX.sym("adaptive_gains", 4, 1)
        self.compute_error_f = self.compute_error(previous_state, state_des, state_meas, adaptive_gains)
        
        error_gradient = cs.jacobian(self.compute_error_f, adaptive_gains)
        self.error_gradient_f = cs.Function("error_gradient", [previous_state, state_des, state_meas, adaptive_gains], [error_gradient])


    def calculate_discrete_LQR_gain(self,lin_state, lin_tau, horizon):
        """Calculate by backward iterations the optimal LQR gains
        Args:
            lin_state (np.array): linearization state
            lin_tau (np.array): linearization control inputs
        Returns:
             K (np.array): optimal gains
        """
        A = self.twip.A_f(lin_state, lin_tau)
        B = self.twip.B_f(lin_state, lin_tau)

 

        A_discrete = A*self.dt/10. + np.identity(6)
        B_discrete = B*self.dt/10.

        for i in range(0, horizon):
            Q_uu = self.R + B_discrete.T@self.P_next@B_discrete
            temp = (-np.linalg.pinv(Q_uu)@B_discrete.T@self.P_next@A_discrete)
            self.P_next = self.Q + A_discrete.T@self.P_next@A_discrete - temp.T@Q_uu@temp
            
        self.K = (np.linalg.pinv(self.R + B_discrete.T@self.P_next@B_discrete)@B_discrete.T@self.P_next@A_discrete)
        print("K", self.K)

        return self.K





    def compute_error(self, previous_state, state_des, state_meas, adaptive_gains):
        """Compute error signal
        Args:
            previous_state (np.array): state at timestep K-1
            state_des (np.array): state_des at timestep K-1
            state_meas (np.array): state measured at time K
            adaptive_gains (np.array)
        Returns:
            (np.array): error signal
        """

        control = self.K@(state_des - previous_state) # to add adaptive gain
        error = state_des - previous_state
        
        control[0] += adaptive_gains[0][0]*error[1]*0 + adaptive_gains[1][0]*error[3] + adaptive_gains[2][0]*error[4]*0.01 + adaptive_gains[3][0]*error[5]
        control[1] += adaptive_gains[0][0]*error[1]*0 + adaptive_gains[1][0]*error[3] + adaptive_gains[2][0]*error[4]*0.01 + adaptive_gains[3][0]*error[5]
        #control[0] += adaptive_gains[1][0]*error[1]
        cs.reshape(control, 2, 1)
        
        next_state = self.twip.forward_dynamics(previous_state, control)
        qdd = next_state[3:6]
        state_pred = self.twip.euler_integration_cs(previous_state, qdd, self.dt)

          
        #should be n_datapoints x num_features
        error_prediction = state_meas[1:] - state_pred[1:]
        #error_prediction = state_pred[1:] - state_meas[1:]
        #error_prediction = state_des[1:] - state_pred[1:] 

        return cs.reshape(error_prediction,1,5)
        #return cs.reshape(next_state[1:],1,5)
    



    
    def compute_adaptive_gains(self, previous_state, state_des, state_meas):
        """Compute a single adaptive gains update
        Args:
            previous_state (np.array): state at timestep K-1
            control (np.array): control at timestep K-1
            state_meas (np.array): state measured at time K  
        """

        error = self.compute_error(previous_state, state_des, state_meas, cs.reshape(self.adaptive_gains,4,1))
        error_grad = self.error_gradient_f(previous_state, state_des, state_meas, cs.reshape(self.adaptive_gains,4,1))
        error_np = np.array([error[0].__float__(), error[1].__float__(), error[2].__float__(), error[3].__float__(), error[4].__float__()])

        
        self.adaptive_gains = self.adaptive_gains - self.gamma*error_np.reshape(1,5)@error_grad

        #self.adaptive_gains = cs.np.array([[self.adaptive_gains[0].__float__(), self.adaptive_gains[1].__float__(), self.adaptive_gains[2].__float__(), self.adaptive_gains[3].__float__()]])
        #self.adaptive_gains = cs.np.reshape(self.adaptive_gains, (4,1))


    
    def batch_adaptive_gains(self, previous_state, control, state_meas):
        """Compute a batch adaptive gains update
        Args:
            previous_state (np.array): state at timestep K-1
            control (np.array): control at timestep K-1
            state_meas (np.array): state measured at time K  
        """
        for i in range(0, previous_state.shape[0]):
            state_pred_lift, error = self.compute_error(previous_state[i], control[i], state_meas[i])


            if(not np.any(self.error_vec)):
                self.error_vec = error
            else:
                self.error_vec = np.append(self.error_vec, error, axis=0)



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

        
        
        control = u_ff + self.K@(state_des - state)
        error = state_des - state
        control[0] += self.adaptive_gains[0][0]*error[1]*0 + self.adaptive_gains[1][0]*error[3] + self.adaptive_gains[2][0]*error[4] - self.adaptive_gains[3][0]*error[5]
        control[1] += self.adaptive_gains[0][0]*error[1]*0 + self.adaptive_gains[1][0]*error[3] + self.adaptive_gains[2][0]*error[4] + self.adaptive_gains[3][0]*error[5]


        return control




if __name__=="__main__":
    dt = 0.01
    controller = Adaptive_LQR(dt=dt)

    state = np.array([0, 0.1, 0, 0.1, 0.1, 0])
    old_state_robot = state
    state_des = np.array([0, 0, 0, 0, 0., 0.])
    state_evolution = [copy.copy(state)]

    robot = Robot_Model()

    for j in range(0, 2000):
        controller.compute_adaptive_gains(old_state_robot, state_des, state)
        control = controller.compute_control(state, state_des)
        tau = np.array([control[0], control[1]]).reshape(controller.control_dim,)

        old_state_robot = state
        qdd = robot.forward_dynamics(state.reshape(controller.state_dim,), tau)
        qdd = qdd[3:6]
        state = robot.euler_integration(state, qdd, dt).reshape(controller.state_dim,)
        state_evolution = np.append(state_evolution, [copy.copy(state)], axis=0)
            

    # Plotting ---------------------------------------
    fig, axs = plt.subplots(2, 2)
    fig.set_figheight(8)
    fig.set_figwidth(10)
    # Defining custom 'xlim' and 'ylim' values.
    custom_xlim = (0, 100)
    custom_ylim = (-2, 2)
    # Setting the values for all axes.
    plt.setp(axs, xlim=custom_xlim, ylim=custom_ylim)
    axs[0, 0].plot(state_evolution[:,1])
    axs[0, 0].set_title('pitch')
    axs[0, 1].plot(state_evolution[:,4])
    axs[0, 1].set_title('pitch_d')
    axs[1, 0].plot(state_evolution[:,3])
    axs[1, 0].set_title('x_d')
    axs[1, 1].plot(state_evolution[:,5])
    axs[1, 1].set_title('yaw_d')
    plt.show()




import numpy as np

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))
import sys
sys.path.append(dir_path + '/./../')
from robot_model import Robot_Model



class LS_LQR:
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

        self.Q = np.zeros((6, 6))
        
        self.Q[0,0] = 0.0 #x
        self.Q[3,3] = 2 #x_d
        self.Q[2,2] = 0.0 #yaw
        
        self.Q[1,1] = 5.0 #pitch
        self.Q[4,4] = 1.0 #pitch_d
        self.Q[5,5] = 1 #yawd

        self.R = np.identity(2)*10

        self.P_next = np.identity(6)
        self.P_next[0,0] = 0.0 #x
        self.P_next[2,2] = 0.0 #yaw


        self.phi_vec = np.array([])
        self.error_vec = np.array([])
        self.basis = 9
        self.best_param = np.zeros((self.basis, 5))


        self.P_least_square = np.array([])
        
        self.K = self.calculate_discrete_LQR_gain(self.lin_state, self.lin_tau, self.horizon)

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

        #state_pred_lift = self.lift_space(lin_state)

        #A_ls = np.zeros((12, 12))
        #A_ls[0:6,0:6] = A
        #A_ls[0:6,7:12] = self.best_param.T@state_pred_lift



        #B_ls = np.zeros((12, 2))
        #B_ls[0:6,:] = B

        
        #A_correction = np.zeros((6,6))
        #A_correction[1:,1:] = self.best_param.T
 

        A_discrete = A*self.dt + np.identity(6)
        B_discrete = B*self.dt

        #print("A_discrete", A_discrete)
        #print("A_correction", A_correction)

        #A_discrete += A_correction

        #print("A_discrete corrected", A_discrete)


        for i in range(0, horizon):
            Q_uu = self.R + B_discrete.T@self.P_next@B_discrete
            temp = (-np.linalg.pinv(Q_uu)@B_discrete.T@self.P_next@A_discrete)
            self.P_next = self.Q + A_discrete.T@self.P_next@A_discrete - temp.T@Q_uu@temp
            
        self.K = (np.linalg.pinv(self.R + B_discrete.T@self.P_next@B_discrete)@B_discrete.T@self.P_next@A_discrete)

        return self.K



    def lift_space(self, state):
        """Nonlinear features for least square
        Args:
            state (np.array): state to lift 
        Returns:
            (np.array): nonlinear state
        """
        return np.array([np.cos(state[1]), np.sin(state[2]), state[1]*state[1], 1, state[3]*state[3], state[4]*state[4], state[5]*state[5], np.cos(state[4]*state[4]), np.sin(state[4]*state[4])])



    def compute_error_and_lift(self, previous_state, control, state_meas):
        """Compute error signal and lift the state
        Args:
            previous_state (np.array): state at timestep K-1
            control (np.array): control at timestep K-1
            state_meas (np.array): state measured at time K  
        Returns:
            (np.array, np.array): nonlinear state, error signal
        """
        print("control", control)
        print("previous_state", previous_state)
        next_state = self.twip.forward_dynamics(previous_state, control)
        qdd = next_state[3:6]
        state_pred = self.twip.euler_integration(previous_state, qdd, self.dt)
        
        # should be 1 x num_features
        state_pred_lift = self.lift_space(state_pred)
        
        #should be n_datapoints x num_features
        error = state_meas - state_pred[1:]
        error = error.reshape(1,5)  



        return state_pred_lift.reshape(1,self.basis), error
        #return state_pred[1:].reshape(1,5), error


    
    def recursive_least_square(self, previous_state, control, state_meas):
        """Compute a recursive least square update
        Args:
            previous_state (np.array): state at timestep K-1
            control (np.array): control at timestep K-1
            state_meas (np.array): state measured at time K  

        """
        state_pred_lift, error = self.compute_error_and_lift(previous_state, control, state_meas)

        if(self.P_least_square.size == 0):
            self.P_least_square = np.linalg.pinv(state_pred_lift.T@state_pred_lift)
            self.best_param = self.P_least_square@state_pred_lift.T@error
        else:
            self.P_least_square = self.P_least_square - self.P_least_square@state_pred_lift.T@np.linalg.pinv(np.identity(1) + state_pred_lift@self.P_least_square@state_pred_lift.T)@state_pred_lift@self.P_least_square
            K_least_square = self.P_least_square@state_pred_lift.T
            self.best_param = self.best_param + K_least_square@(error)

        


    
    def full_least_square(self, previous_state, control, state_meas):
        """Compute a full least square update
        Args:
            previous_state (np.array): state at timestep K-1
            control (np.array): control at timestep K-1
            state_meas (np.array): state measured at time K  
        """
        for i in range(0, previous_state.shape[0]):
            state_pred_lift, error = self.compute_error_and_lift(previous_state[i], control[i], state_meas[i])

            if(not np.any(self.phi_vec)):
                self.phi_vec = state_pred_lift
            else:
                self.phi_vec = np.append(self.phi_vec, state_pred_lift, axis=0)

            if(not np.any(self.error_vec)):
                self.error_vec = error
            else:
                self.error_vec = np.append(self.error_vec, error, axis=0)


        self.P_least_square = np.linalg.pinv(self.phi_vec.T@self.phi_vec + 0.4*np.identity(self.phi_vec.shape[1]))
        self.best_param = self.P_least_square@self.phi_vec.T@self.error_vec



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

        #horizon = 10
        #self.K = self.calculate_discrete_LQR_gain(state_des, u_ff, horizon)

        state_des_lift = self.lift_space(state_des)
        state_lift = self.lift_space(state)

        #print("best param", self.best_param)
        

        return u_ff + self.K@(state_des - state)




if __name__=="__main__":
    control = LS_LQR(dt=0.01, horizon=2000)
    
    x1 = np.array([0, 0, 0, 0, 1., 0.])
    u1 = np.array([0.1, 0.1])
    y1_meas = np.array([0, 0, 0, 0.95, 0])

    x2 = np.array([0, 0, 0, 0, 0.95, 0.])
    u2 = np.array([0.1, 0.1])
    y2_meas = np.array([0, 0, 0, 1.9, 0])


    x3 = np.array([2, 2, 0, 0, 0.95, 0.])
    u3 = np.array([0.1, 0.1])
    y3_meas = np.array([2, 0, 0, 1.9, 0])

    x = np.vstack((x1,x2))
    u = np.vstack((u1,u2))
    y_meas = np.vstack((y1_meas,y2_meas))

    x = np.vstack((x,x3))
    u = np.vstack((u,u3))
    y_meas = np.vstack((y_meas,y3_meas))

    print("x.shape", x.shape)
    print("u.shape", u.shape)
    print("y_meas.shape", y_meas.shape)

    control.full_least_square(x, u, y_meas)

    #control.recursive_least_square(x1, u1, y1_meas)

    print("self.best_param", control.best_param.shape)

    #control.calculate_discrete_LQR_gain(control.lin_state, control.lin_tau, 20)

    x2 = np.array([0, 0, 0, 0, 0.95, 0.])
    u2 = np.array([0.1, 0.1])
    y2_meas = np.array([0, 0, 0, 1.9, 0])
    control.recursive_least_square(x2, u2, y2_meas)

    
    
    

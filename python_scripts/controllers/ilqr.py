import control
import numpy as np

import sys
sys.path.append('/home/python_scripts/')
import forward_dynamics
import euler_integration
from twip_dynamics import Twip_dynamics

import matplotlib.pyplot as plt # type: ignore

import time

import copy

class iLQR:
    def __init__(self, lin_state = None, lin_tau = None, horizon = None, dt = None):
        self.lin_state = np.zeros(6)
        self.lin_tau = np.zeros(2)
        self.horizon = 15
        self.iteration = 2
        self.dt = dt

        self.twip = Twip_dynamics()

        self.state_dim = 6
        self.control_dim = 2

        self.Q = np.identity(6)
        
        self.Q[0,0] = 0.0 #x
        self.Q[3,3] = 2 #x_d
        
        self.Q[2,2] = 0.0 #yaw
        self.Q[5,5] = 0.2 #yawd

        self.Q[1,1] = 5.0 #pitch
        self.Q[4,4] = 1.0 #pitch_d
        

        self.R = np.identity(2)*2
        
        # compute optimal cost to go LQR
        self.P = self.compute_discrete_LQR_P(self.lin_state, self.lin_tau)


        self.P_vec = np.zeros((self.horizon+1, self.state_dim, self.state_dim));
        self.P_vec[self.horizon] = self.P
        self.V_vec = np.zeros(((self.horizon+1),self.state_dim,1));
    

        self.state_vec = np.zeros(((self.horizon+1), self.state_dim, 1));
        self.control_vec = np.zeros(((self.horizon), self.control_dim, 1));


        self.A_vec = np.zeros((self.horizon, self.state_dim, self.state_dim));
        self.B_vec = np.zeros((self.horizon, self.state_dim, self.control_dim));

        self.Q_uu_vec = np.zeros((self.horizon, self.control_dim, self.control_dim));
        self.pinv_Q_uu_vec = np.zeros((self.horizon, self.control_dim, self.control_dim));
        self.Q_ux_vec = np.zeros((self.horizon, self.control_dim, self.state_dim));
        self.Q_u_vec = np.zeros((self.horizon, self.control_dim, 1));


    def compute_discrete_LQR_P(self,lin_state, lin_tau):

        P_next = np.identity(self.state_dim)

        A = forward_dynamics.compute_A_matrix(lin_state, lin_tau)
        B = forward_dynamics.compute_B_matrix(lin_state, lin_tau)

        A_discrete = A*self.dt + np.identity(self.state_dim)
        B_discrete = B*self.dt


        for i in range(0, self.horizon):
            Q_uu = self.R + B_discrete.T@P_next@B_discrete

            temp = (-np.linalg.pinv(Q_uu)@B_discrete.T@P_next@A_discrete)
            P_next = self.Q + A_discrete.T@P_next@A_discrete - temp.T@Q_uu@temp
            
            self.K = (np.linalg.pinv(self.R + B_discrete.T@P_next@B_discrete)@B_discrete.T@P_next@A_discrete)

        return P_next
    
    def compute_backward_pass(self, state_des):
        #print("##BACKWATD PASS")
        

        for step in range(0,self.horizon):
            state_actual = self.state_vec[self.horizon-step];
            control_actual = self.control_vec[self.horizon-step-1];

            # compute discrete A and B matrices
            A_step = self.twip.A_f(state_actual, control_actual)
            A_step = np.array(A_step)
            B_step = self.twip.B_f(state_actual, control_actual)
            B_step = np.array(B_step)

            A_step = A_step*self.dt + np.identity(self.state_dim)
            B_step = B_step*self.dt
            self.A_vec[self.horizon - step - 1] = A_step
            self.B_vec[self.horizon - step - 1] = B_step

    
            # calculate P - optimal cost to go
            P_step = self.P_vec[self.horizon - step]
            
            # calculate Q_uu and pinv_Q_uu
            temp = B_step.T@P_step
            Q_uu = self.R + temp@B_step
            pinv_Q_uu = np.linalg.pinv(Q_uu)

            # calculate Q_ux 
            Q_ux = temp@A_step
            
            # calculate optimal cost to go at the previous step
            temp_1 = (-pinv_Q_uu@Q_ux)
            P_step = self.Q + A_step.T@P_step@A_step - temp_1.T@Q_uu@temp_1
            
            # calculate Q_u, V and v
            u = self.control_vec[self.horizon-step-1] 
            V = self.V_vec[self.horizon - step]
            Q_u = (self.R@u) + B_step.T@V
            v = self.Q@(state_actual - state_des) + A_step.T@V - temp_1.T@Q_uu@(-pinv_Q_uu@Q_u);

            # save everything
            self.Q_uu_vec[self.horizon - step - 1] = Q_uu
            self.pinv_Q_uu_vec[self.horizon - step - 1] = pinv_Q_uu
            self.Q_ux_vec[self.horizon - step - 1] = Q_ux
            self.Q_u_vec[self.horizon-step-1] = Q_u

            self.P_vec[self.horizon - step - 1] = P_step
            self.V_vec[self.horizon - step - 1] = v.reshape(self.state_dim,1);
        

        


    def compute_forward_pass(self,initial_state):

        self.state_vec[0] = initial_state.reshape(6,1) 
        state_forward = copy.deepcopy(self.state_vec)
        
        for step in range(0,self.horizon):

            start_time = time.time()
            
            error = (self.state_vec[step] - state_forward[step])

            # taking value from backward pass
            pinv_Q_uu = self.pinv_Q_uu_vec[step]
            Q_ux = self.Q_ux_vec[step]
            Q_u = self.Q_u_vec[step]


            # new control update
            u_update_k = -pinv_Q_uu@Q_u
            u_update_K = -pinv_Q_uu@Q_ux@(error)
            
            self.control_vec[step,0] += u_update_k[0]*1 + u_update_K[0]
            self.control_vec[step,1] += u_update_k[1]*1 + u_update_K[1]
            
            
            #print("forward time first: ", time.time()-start_time)
            start_time = time.time()

            # simulate system
            state = self.state_vec[step]
            state = state.reshape(self.state_dim,)
            control = self.control_vec[step]
            control = control.reshape(self.control_dim,)
            #next_state = forward_dynamics.forward_dynamics(state,control);
            next_state = self.twip.fd(state,control); 
            qdd = next_state[3:6]

            #print("forward time second: ", time.time()-start_time)


            # integration
            #print("state forward", state)
            self.state_vec[step+1] = euler_integration.euler_integration(state, qdd, self.dt).reshape(self.state_dim,1)

            
            
            '''#cost
            cost_temp = cost_temp + error'*Q*error + [u_l(1,step);u_r(1,step)]'*R*[u_l(1,step);u_r(1,step)];'''

        #print("evolution forward pass", self.state_vec)


    def compute_forward_simulation(self,initial_state):

        self.state_vec[0] = initial_state.reshape(6,1) 

        for step in range(0,self.horizon):

            # simulate system
            state = self.state_vec[step]
            state = state.reshape(self.state_dim,)
            control = self.control_vec[step]
            control = control.reshape(self.control_dim,)
            qdd = self.twip.fd(state,control); 
            qdd = qdd[3:6]

            # integration
            self.state_vec[step+1] = euler_integration.euler_integration(state, qdd, self.dt).reshape(self.state_dim,1)

            


    def compute_control(self, state, state_des):
        state_des[1] = forward_dynamics.compute_angle_from_vel(state_des[3])
        state_des = state_des.reshape(self.state_dim,1)

        # setting last V and initial system simulation
        self.V_vec[self.horizon] = self.P@(state.reshape(self.state_dim,1) - state_des)
        controller.compute_forward_simulation(initial_state=state)
        
        # compute control and gain
        for i in range(0,self.iteration):
            start_time = time.time()
            controller.compute_backward_pass(state_des=state_des)
            #print("backward time: ", time.time()-start_time)
            start_time = time.time()
            controller.compute_forward_pass(initial_state=state)
            #print("forward time: ", time.time()-start_time)
        
        print("control vec", self.control_vec)

        return [0,0]





if __name__=="__main__":
    controller=iLQR(dt = 0.01)
    state = np.zeros(6)
    state[1] = 0.1 
    state_des = np.zeros(6)
    tau = np.zeros(2)
    start_time = time.time()
    controller.compute_control(state, state_des=state_des)
    print("Control time: ", time.time()-start_time)

    plt.plot(controller.state_vec[:,4])
    plt.show()

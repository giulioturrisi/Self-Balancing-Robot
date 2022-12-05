import numpy as np
import casadi as cs


class Twip_dynamics:
    def __init__(self,):

        #robot parameters
        self.m_b = 1.0 #mass base - kg
        self.m_w = 0.25 #wheel mass - kg
        self.I_pitch = 2.167e-02 #inertia pitch
        self.I_roll = 2.167e-02 #inertia roll
        self.I_yaw = 4.167e-03
        self.I_w1 = 5.000e-03 #inertial wheel axis
        self.I_w2 = 2.575e-03 #inertial wheel vertical axis

        self.r = 0.1 #radius wheels
        self.d = 0.2  #distance between wheels
        self.g = 9.8 #gravity
        self.l = 0.2 #lenght body
        self.c_alpha = -0.001 #viscous friction

        state = cs.SX.sym("state", 6, 1)
        tau = cs.SX.sym("tau", 2, 1)
        forward_dynamics_f = self.forward_dynamics(state, tau)
        self.fd = cs.Function("fd", [state, tau], [forward_dynamics_f])

        self.A_f = self.get_A_f_matrix(state, tau)
        self.B_f = self.get_B_f_matrix(state, tau)


    def forward_dynamics(self, state, tau):
        #x, pitch, yaw, xd, pitch_d, yaw_d
        x = state[0] 
        pitch = state[1]
        yaw = state[2]

        x_d = state[3]
        pitch_d = state[4]
        yaw_d = state[5]
        
        tau_l = tau[0]    
        tau_r = tau[1]

        a11 = self.m_b + 2*self.m_w + 2*self.I_w1/(self.r*self.r)
        a12 = self.m_b*self.l*cs.np.cos(pitch)
        a21 = a12
        a22 = self.I_roll + self.m_b*self.l*self.l
        a33 = self.I_pitch + 2*self.I_w2 + (self.m_w + self.I_w1/(self.r*self.r))*(self.d*self.d/2) - (self.I_roll - self.I_yaw - self.m_b*self.l*self.l)*cs.np.sin(pitch)*cs.np.sin(pitch)
        c12 = -self.m_b*self.l*pitch_d*cs.np.sin(pitch)
        c13 = -self.m_b*self.l*yaw_d*cs.np.sin(pitch)
        c23 = (self.I_yaw - self.I_roll - self.m_b*self.l*self.l)*yaw_d*cs.np.sin(pitch)*cs.np.cos(pitch)
        c31 = self.m_b*self.l*yaw_d*cs.np.sin(pitch)
        c32 = -(self.I_yaw - self.I_roll - self.m_b*self.l*self.l)*yaw_d*cs.np.sin(pitch)*cs.np.cos(pitch)
        c33 = -(self.I_yaw - self.I_roll - self.m_b*self.l*self.l)*pitch_d*cs.np.sin(pitch)*cs.np.cos(pitch)
        d11 = 2*self.c_alpha/(self.r*self.r)
        d12 = -2*self.c_alpha/self.r
        d21 = d12
        d22 = 2*self.c_alpha
        d33 = ((self.d*self.d)/(2*self.r*self.r))*self.c_alpha

        M = cs.np.matrix([[a11, a12, 0], [a21, a22, 0], [0, 0, a33]])  
        C = cs.np.matrix([[0, c12, c13], [0, 0, c23], [c31, c32, c33]])
        B = cs.np.matrix([[1/self.r, 1/self.r], [-1, -1], [-self.d/(2*self.r), self.d/(2*self.r)]])
        G = cs.np.array([[0, -self.m_b*self.l*self.g*cs.np.sin(pitch), 0]]).T
        D = cs.np.matrix([[d11, d12, 0], [d21, d22, 0], [0, 0, d33]])

        tau = cs.np.array([[tau_l, tau_r]]).T
        qd = cs.np.array([[x_d, pitch_d, yaw_d]]).T
        qdd = cs.inv(M)@(-C@qd - G - D@qd + B@tau) 


        return cs.vertcat(qd,qdd)

    def inv_control_matrix(self,):
        B = np.matrix([[1/self.r, 1/self.r], [-1, -1], [-self.d/(2*self.r), self.d/(2*self.r)]])
        return np.linalg.pinv(B)

    def compute_feed_forward(self, pitch,vel):
        #return m_b*l*g*np.sin(pitch)/2.
        return (self.m_b*self.l*self.g*np.sin(pitch) + 2*self.c_alpha*vel/self.r)/2.

    def compute_angle_from_vel(self,vel):
        #return -m_b*l*g*np.sin(pitch) -2*c_alpha*vel/r
        return np.arcsin(2*self.c_alpha*vel/self.r)/(-self.m_b*self.l*self.g)


    def compute_A_matrix(self,state, tau):
        state_sym = cs.SX.sym("state", 6, 1)
        tau_sym = cs.SX.sym("tau", 2, 1)
        forward_dynamics_f = self.forward_dynamics(state_sym, tau_sym)
        #fd = cs.Function("fd", [state_sym, tau_sym], [forward_dynamics_f])

        A = cs.jacobian(forward_dynamics_f, state_sym)
        A_f = cs.Function("A", [state_sym, tau_sym], [A])

        return np.array(A_f(state, tau))



    def compute_B_matrix(self, state, tau):
        state_sym = cs.SX.sym("state", 6, 1)
        tau_sym = cs.SX.sym("tau", 2, 1)
        forward_dynamics_f = self.forward_dynamics(state_sym, tau_sym)
        #fd = cs.Function("fd", [state, tau], [forward_dynamics_f])

        B = cs.jacobian(forward_dynamics_f, tau_sym)
        B_f = cs.Function("B", [state_sym, tau_sym], [B])

        return np.array(B_f(state, tau))


    def get_A_f_matrix(self,state, tau):
        state_sym = cs.SX.sym("state", 6, 1)
        tau_sym = cs.SX.sym("tau", 2, 1)
        forward_dynamics_f = self.forward_dynamics(state_sym, tau_sym)
        #fd = cs.Function("fd", [state_sym, tau_sym], [forward_dynamics_f])

        A = cs.jacobian(forward_dynamics_f, state_sym)
        A_f = cs.Function("A", [state_sym, tau_sym], [A])

        return A_f



    def get_B_f_matrix(self, state, tau):
        state_sym = cs.SX.sym("state", 6, 1)
        tau_sym = cs.SX.sym("tau", 2, 1)
        forward_dynamics_f = self.forward_dynamics(state_sym, tau_sym)
        #fd = cs.Function("fd", [state, tau], [forward_dynamics_f])

        B = cs.jacobian(forward_dynamics_f, tau_sym)
        B_f = cs.Function("B", [state_sym, tau_sym], [B])

        return B_f

    def forward_dynamics_integral(self, state, reference, tau):
        forward_dynamics_output =  self.forward_dynamics(state[0:6],tau)
        #print("forward dynamic output", forward_dynamics_output)
        #print("reference - state[6]", reference - state[6])
        #qd = cs.vertcat(forward_dynamics_output[0:3], reference - state[1])
        #print("qd", qd)
        qd = forward_dynamics_output[0:3]
        qdd = cs.vertcat(forward_dynamics_output[3:], state[3] - reference)
        #print("qdd", qdd)
        return cs.vertcat(qd,qdd)


    def compute_A_matrix_integral(self, state, reference, tau):
        state_sym = cs.SX.sym("state", 7, 1)
        tau_sym = cs.SX.sym("tau", 2, 1)
        reference_sym = cs.SX.sym("reference", 1, 1)
        forward_dynamics_f = self.forward_dynamics_integral(state_sym, reference_sym, tau_sym)
        #fd = cs.Function("fd", [state_sym, tau_sym], [forward_dynamics_f])

        A = cs.jacobian(forward_dynamics_f, state_sym)
        A_f = cs.Function("A", [state_sym, tau_sym], [A])

        return np.array(A_f(state, tau))



    def compute_B_matrix_integral(self, state, reference, tau):
        state_sym = cs.SX.sym("state", 7, 1)
        tau_sym = cs.SX.sym("tau", 2, 1)
        reference_sym = cs.SX.sym("reference", 1, 1)
        forward_dynamics_f = self.forward_dynamics_integral(state_sym, reference_sym, tau_sym)
        #fd = cs.Function("fd", [state, tau], [forward_dynamics_f])

        B = cs.jacobian(forward_dynamics_f, tau_sym)
        B_f = cs.Function("B", [state_sym, tau_sym], [B])

        return np.array(B_f(state, tau))





if __name__=="__main__":
    forward_dynamics(np.zeros(6), np.zeros(2))

    
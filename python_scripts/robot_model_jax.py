import jax
import jax.numpy as jnp

class Robot_Model_Jax:
    """This class computes the forward dynamics, the linearization matrices, and other minor quantities
       based on the model for the two wheeled inverted pendulum 
    """


    def __init__(self,):
        """
        Args: Nothing!
        """
        # Robot parameters -----------------------------------------
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


    def forward_dynamics(self, state, tau):
        """Compute forward dynamics

        Args:
            state (np.array): state of the robot at time K-1
            tau (np.array): control applied to the robot at time K-1

        Returns:
            (np.array): instantaneous acceleration of the system and velocity
        """
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
        a12 = self.m_b*self.l*jnp.cos(pitch)
        a21 = a12
        a22 = self.I_roll + self.m_b*self.l*self.l
        a33 = self.I_pitch + 2*self.I_w2 + (self.m_w + self.I_w1/(self.r*self.r))*(self.d*self.d/2) - (self.I_roll - self.I_yaw - self.m_b*self.l*self.l)*jnp.sin(pitch)*jnp.sin(pitch)
        c12 = -self.m_b*self.l*pitch_d*jnp.sin(pitch)
        c13 = -self.m_b*self.l*yaw_d*jnp.sin(pitch)
        c23 = (self.I_yaw - self.I_roll - self.m_b*self.l*self.l)*yaw_d*jnp.sin(pitch)*jnp.cos(pitch)
        c31 = self.m_b*self.l*yaw_d*jnp.sin(pitch)
        c32 = -(self.I_yaw - self.I_roll - self.m_b*self.l*self.l)*yaw_d*jnp.sin(pitch)*jnp.cos(pitch)
        c33 = -(self.I_yaw - self.I_roll - self.m_b*self.l*self.l)*pitch_d*jnp.sin(pitch)*jnp.cos(pitch)
        d11 = 2*self.c_alpha/(self.r*self.r)
        d12 = -2*self.c_alpha/self.r
        d21 = d12
        d22 = 2*self.c_alpha
        d33 = ((self.d*self.d)/(2*self.r*self.r))*self.c_alpha

        M = jnp.array([[a11, a12, 0], [a21, a22, 0], [0, 0, a33]])  
        C = jnp.array([[0, c12, c13], [0, 0, c23], [c31, c32, c33]])
        B = jnp.array([[1/self.r, 1/self.r], [-1, -1], [-self.d/(2*self.r), self.d/(2*self.r)]])
        G = jnp.array([[0, -self.m_b*self.l*self.g*jnp.sin(pitch), 0]]).T
        D = jnp.array([[d11, d12, 0], [d21, d22, 0], [0, 0, d33]])

        tau = jnp.array([[tau_l, tau_r]]).T
        qd = jnp.array([[x_d, pitch_d, yaw_d]]).T
        qdd = jnp.linalg.pinv(M)@(-C@qd - G - D@qd + B@tau) 
        return jnp.vstack([qd,qdd])


    def euler_integration(self, state, qdd, dt):
        """Small function that compute a simple euler integration

        Args:
            state (np.array): state of the robot at tike K
            qdd (np.array): instantaneous acceleration of the system
            dt (float): sampling time

        Returns:
            (np.array): new state of the robot at time K+1
        """
        #x, pitch, yaw, xd, pitch_d, yaw_d

        x = state[0] 
        x_dot = state[3]
        
        pitch = state[1]
        pitch_dot = state[4]
        
        yaw = state[2]
        yaw_dot = state[5]
        
        x = x + x_dot*dt
        x_dot = x_dot + qdd[0][0]*dt
        #x = x + x_dot*dt

        pitch = pitch + pitch_dot*dt
        pitch_dot = pitch_dot + qdd[1][0]*dt
        #pitch = pitch + pitch_dot*dt

        yaw = yaw + yaw_dot*dt
        yaw_dot = yaw_dot + qdd[2][0]*dt
        #yaw = yaw + yaw_dot*dt

        state = jnp.array([x, pitch, yaw, x_dot, pitch_dot, yaw_dot])
        state = state.reshape(6,)

        return state




if __name__=="__main__":
    Robot_Model_Jax()

    
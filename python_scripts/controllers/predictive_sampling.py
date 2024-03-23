import jax
import jax.numpy as jnp
from jax import jit
from jax import random
import os
#os.environ["XLA_PYTHON_CLIENT_ALLOCATOR"] = "platform"
os.environ["XLA_PYTHON_CLIENT_MEM_FRACTION"] = ".75"
#jax.config.update('jax_platform_name', 'cpu')

import numpy as np
import matplotlib.pyplot as plt #
import time

import copy 

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))
import sys
sys.path.append(dir_path + '/./../')
from robot_model import Robot_Model
from robot_model_jax import Robot_Model_Jax




class Sampling_MPC:
    """This is a small class that implements a sampling based control law"""


    def __init__(self, horizon = 200, dt = 0.01, num_computations = 1000, init_jax = True, linear = True):
        """
        Args:
            horizon (int): how much to look into the future for optimizing the gains 
            dt (int): desidered sampling time
        """
        self.horizon = horizon
        self.dt = dt
        self.state_dim = 6
        self.control_dim = 2
        self.num_computations = num_computations
        self.max_input = 0.5

        if(linear == True):
            self.spline_fun = self.compute_linear_spline
        else:
            self.spline_fun = self.compute_cubic_spline 

        self.robot =  Robot_Model_Jax()

        self.Q = jnp.identity(self.state_dim)
        

        self.R = jnp.identity(self.control_dim)

        self.num_parameters = 16

        # the first call of jax is very slow, hence we should do this since the beginning! ------------------
        if(init_jax):
            vectorized_forward_sim = jax.vmap(self.compute_forward_simulations, in_axes=(0,0,0), out_axes=0)
            self.jit_vectorized_forward_sim = jax.jit(vectorized_forward_sim)
            
            threads = self.num_computations
            
            #x, pitch, yaw, xd, pitch_d, yaw_d
            reference_x = []
            reference_pitch = []
            reference_yaw = []
            reference_x_d = []
            reference_pitch_d = []
            reference_yaw_d = []
            for i in range(self.horizon):
                    reference_x.append(0.0)
                    reference_pitch.append(0.0)
                    reference_yaw.append(0.0)
                    reference_x_d.append(0.0)
                    reference_pitch_d.append(0.0)
                    reference_yaw_d.append(0.0)
            
            state_des = jnp.column_stack([reference_x, reference_pitch, reference_yaw, reference_x_d, reference_pitch_d, reference_yaw_d])
            xs_des = jnp.tile(state_des, (self.num_computations,1)).reshape(self.num_computations, self.horizon, self.state_dim)
            xs = jnp.zeros((self.state_dim*threads,)).reshape(threads,self.state_dim)
            
            key = random.PRNGKey(42)
            parameters_map = random.randint(key,(self.num_parameters*threads,), minval=-200, maxval=200 )/100.
            self.parameters_map = parameters_map.reshape(threads,self.num_parameters)
            
            self.jit_vectorized_forward_sim(xs, xs_des, self.parameters_map)

            
    def reset(self,):
        """Every control class should have a reset function
        """
        return
    
    
    def compute_linear_spline(self, parameters, step):
        index = 0
        index = jax.numpy.where(step > self.horizon/4, 2, index)
        index = jax.numpy.where(step > self.horizon/2, 4, index)
        index = jax.numpy.where(step > self.horizon-10, 6, index) 

        q = (step*0.01 - 0)/(self.horizon*0.01)
        tau_l = (1-q)*parameters[index+0] + q*parameters[index+1]
        tau_r = (1-q)*parameters[index+8] + q*parameters[index+9]

        return tau_l, tau_r
    
    def compute_cubic_spline(self, parameters, step):
        
        q = (step*0.01 - 0)/(self.horizon*0.01)
        
        phi = (1./2.)*(((parameters[2] - parameters[1])/0.5) + ((parameters[1] - parameters[0])/0.5))
        phi_next = (1./2.)*(((parameters[3] - parameters[2])/0.5) + ((parameters[2] - parameters[1])/0.5))
        
        a_v = 2*q*q*q - 3*q*q + 1
        b_v = (q*q*q - 2*q*q + q)*0.5
        c_v = -2*q*q*q + 3*q*q
        d_v = (q*q*q - q*q)*0.5
        tau_l = a_v*parameters[1] + b_v*phi + c_v*parameters[2] + d_v*phi_next

        phi = (1./2.)*(((parameters[6] - parameters[5])/0.5) + ((parameters[5] - parameters[4])/0.5))
        phi_next = (1./2.)*(((parameters[7] - parameters[6])/0.5) + ((parameters[6] - parameters[5])/0.5))
        
        a_w = 2*q*q*q - 3*q*q + 1
        b_w = (q*q*q - 2*q*q + q)*0.5
        c_w = -2*q*q*q + 3*q*q
        d_w = (q*q*q - q*q)*0.5
        tau_r = a_w*parameters[5] + b_w*phi + c_w*parameters[6] + d_w*phi_next
       
        return tau_l, tau_r


    def compute_forward_simulations(self, initial_state, state_des, parameters):
        """Calculate cost of a rollout of the dynamics given random parameters
        Args:
            initial_state (np.array): actual state of the robot
            state_des (np.array): desired state of the robot
            parameters (np.array): parameters for the controllers
        Returns:
            (float): cost of the rollout
        """
        state = initial_state
        cost = 0.0
    
        def iterate_fun(n, carry):
            cost, state, state_des = carry

            tau_l, tau_r = self.spline_fun(parameters, n)
            #v, w = self.compute_linear_spline(parameters, n)
            #v, w = self.compute_cubic_spline(parameters, n)

            tau_l = jax.numpy.where(tau_l > self.max_input, self.max_input, tau_l)
            tau_l = jax.numpy.where(tau_l < -self.max_input, -self.max_input, tau_l)
            
            tau_r = jax.numpy.where(tau_r > self.max_input, self.max_input, tau_r)
            tau_r = jax.numpy.where(tau_r < -self.max_input, -self.max_input, tau_r)

            control = jnp.array([tau_l, tau_r])
            qdd = self.robot.forward_dynamics(state.reshape(self.state_dim,), control);
            qdd = qdd[3:6]
            state_next = self.robot.euler_integration(state, qdd, self.dt).reshape(self.state_dim,)
            
            error = state_next.reshape(self.state_dim, 1) - state_des[n].reshape(self.state_dim, 1)
            
            #x, pitch, yaw, xd, pitch_d, yaw_d
            cost_next = error[1]*2.0*error[1] + error[4]*0.1*error[4] + error[5]*0.1*error[5]
            cost_next = [cost_next]

            return (cost_next[0][0] + cost, state_next, state_des)

        carry = (cost, state, state_des)
        cost, state, state_des = jax.lax.fori_loop(0, self.horizon, iterate_fun, carry)
        
        return cost
    
    
    def compute_control(self, state, state_des):
        """Compute control inputs
        Args:
            state (np.array): actual robot state
            state_des (np.array): desired robot state
        Returns:
            (np.array): optimized control inputs
        """
        
        state_vec = jnp.tile(state, (self.num_computations,1))
        
        state_des = jnp.tile(state_des, (self.horizon,1))
        state_des_vec = jnp.tile(state_des, (self.num_computations,1)).reshape(self.num_computations, self.horizon, self.state_dim)
        
        #key = random.PRNGKey(42)
        #parameters_map = random.randint(key,(self.num_parameters*1,), minval=-200, maxval=200 )/100.
       
        cost = self.jit_vectorized_forward_sim(state_vec, state_des_vec, self.parameters_map)
        
        best_index = np.nanargmin(cost)
        best_parameters = self.parameters_map[best_index]
        tau_l, tau_r = self.spline_fun(best_parameters, 0)

        return tau_l, tau_r




if __name__=="__main__":
    dt = 0.01
    controller = Sampling_MPC(dt = dt, horizon = 20, init_jax = True, 
                              num_computations = 1000, linear = False)

    state = np.array([1, 0.1, 0.1, 0.1, 0.1, 0.1])
    state_des = np.array([0, 0, 0, 0, 0., 0.])
    state_evolution = [copy.copy(state)]

    robot = Robot_Model()

    
    for j in range(0, 1000):
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



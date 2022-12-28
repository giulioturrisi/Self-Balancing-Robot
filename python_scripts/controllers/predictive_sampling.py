import jax
import jax.numpy as jnp
from jax import jit
from jax import random
import os
#os.environ["XLA_PYTHON_CLIENT_ALLOCATOR"] = "platform"
os.environ["XLA_PYTHON_CLIENT_MEM_FRACTION"] = ".5"
#jax.config.update('jax_platform_name', 'cpu')

import numpy as np
import time

import sys
sys.path.append('/home/python_scripts/')
from twip_dynamics_jax import Twip_dynamics_jax

sys.path.append('/home/python_scripts/controllers')
from lqr import LQR 

import euler_integration
from twip_dynamics import Twip_dynamics

#TO DO: for loops are too slow to compile!

class Sampling_MPC:
    """This is a small class that implements a sampling based control law"""


    def __init__(self, horizon = 200, dt = 0.1, num_computation = 100, init_jax = True):
        """
        Args:
            horizon (int): how much to look into the future for optimizing the gains 
            dt (int): desidered sampling time
        """
        self.horizon = horizon
        self.dt = dt
        self.state_dim = 6
        self.control_dim = 2

        self.twip_jax = Twip_dynamics_jax()

        self.Q = jnp.identity(self.state_dim)
        self.Q.at[0,0].set(0.0)
        self.Q.at[1,1].set(5.0)
        self.Q.at[2,2].set(0.0)
        self.Q.at[3,3].set(2.0)
        self.Q.at[4,4].set(1.0)
        self.Q.at[5,5].set(0.2)
        self.R = jnp.identity(self.control_dim)*10

        # lqr control law used as baseline to analyze performance ---------------------------W----------------
        self.twip = Twip_dynamics()
        self.lqr = LQR(dt = self.dt/10.)


        # the first call of jax is very slow, hence we should do this since the beginning! ------------------
        if(init_jax):
            vectorized_forward_sim = jax.vmap(self.compute_forward_simulations, in_axes=(0,0,0), out_axes=0)
            self.jit_vectorized_forward_sim = jax.jit(vectorized_forward_sim)
            
            threads = num_computation
            
            xs_des = jnp.zeros((6*threads,)).reshape(threads,6)
            xs = jnp.zeros((6*threads,)).reshape(threads,6)
            key = random.PRNGKey(42)
            parameters_map = random.randint(key,(4*threads,), minval=-40, maxval=40 )/20.0
            self.parameters_map = parameters_map.reshape(threads,4)
            
            self.jit_vectorized_forward_sim(xs, xs_des, self.parameters_map)


    def compute_forward_simulations_baseline(self, initial_state, state_des, parameters):
        """Calculate cost of a rollout of the dynamics given an LQR control law
        Args:
            initial_state (np.array): actual state of the robot
            state_des (np.array): desired state of the robot
            parameters (np.array): parameters for the controllers
        Returns:
            (float): cost of the rollout
        """

        state = initial_state
        cost = 0
        
        for step in range(0,self.horizon):
            # simulate system
            error = state_des - state

            control_lqr = self.lqr.K@error
            control = np.array([control_lqr[0].__float__(), control_lqr[1].__float__()])


            qdd = self.twip.forward_dynamics(state.reshape(self.state_dim,), control); 
            qdd = qdd[3:6]
            
            
            # integration
            state = euler_integration.euler_integration(state, qdd, self.dt).reshape(self.state_dim,)
            
            cost += (state.reshape(self.state_dim,1) - state_des.reshape(self.state_dim,1)).T@self.Q@(state.reshape(self.state_dim,1) - state_des.reshape(self.state_dim,1))
        return cost[0][0] 
        
    

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
        cost = 0
        
        for step in range(0,self.horizon):

            # simulate system
            error = state_des - state

            #u_0 = -1.78144*error[1] - 0.300708*error[3]  -0.369117*error[4] -0.100764*error[5]
            #u_1 = -1.78144*error[1] - 0.300708*error[3]  -0.369117*error[4] +0.100764*error[5]
            u_0 = parameters[0]*error[1] + parameters[1]*error[3] + parameters[2]*error[4] - parameters[3]*error[5]
            u_1 = parameters[0]*error[1] + parameters[1]*error[3] + parameters[2]*error[4] + parameters[3]*error[5]
            control = jnp.array([u_0, u_1])
            control = jax.numpy.where(control > 0.5, 0.5, control)
            control = jax.numpy.where(control < -0.5, -0.5, control)

            
            qdd = self.twip_jax.forward_dynamics(state.reshape(self.state_dim,), control);
            qdd = qdd[3:6]
            
            # integration
            state = self.twip_jax.euler_integration(state, qdd, self.dt).reshape(self.state_dim,)
            
            cost += (state.reshape(self.state_dim,1) - state_des.reshape(self.state_dim,1)).T@self.Q@(state.reshape(self.state_dim,1) - state_des.reshape(self.state_dim,1))
        return cost[0][0]
    
    
    
    
    def compute_control(self, state, state_des):
        """Compute control inputs
        Args:
            state (np.array): actual robot state
            state_des (np.array): desired robot state
        Returns:
            (np.array): optimized control inputs
        """
        state_vec = jnp.tile(state, (self.num_computation,1))
        state_des_vec = jnp.tile(state_des, (self.num_computation,1))
        
        cost = self.jit_vectorized_forward_sim(state_vec, state_des_vec, self.parameters_map)
        best_index = np.nanargmin(cost)
        best_parameters = self.parameters_map[best_index]
        
        error = state_des - state
        u_0 = best_parameters[0]*error[1] + best_parameters[1]*error[3] + best_parameters[2]*error[4] - best_parameters[3]*error[5]
        u_1 = best_parameters[0]*error[1] + best_parameters[1]*error[3] + best_parameters[2]*error[4] + best_parameters[3]*error[5]
        control = np.array([u_0, u_1])

        return control




if __name__=="__main__":
    control = Sampling_MPC(dt=0.01, horizon=20, init_jax = False)

    # single computation test ------------------------------------
    
    x = jnp.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    x_des = jnp.array([0, 0, 0, 0, 0., 0.])
    key = random.PRNGKey(42)
    parameters = random.uniform(key,(4,), minval=-1, maxval=1)*3.0
    parameters= parameters.reshape(-1,4)
    #print("parameters: ", parameters)
    #print("K lqr: ", control.lqr.K)
    
    start_time = time.time()
    cost = control.compute_forward_simulations_baseline(x, x_des, parameters[0])
    print("baseline computation: ", time.time()-start_time)
    #print("cost: ", cost)

    start_time = time.time()
    cost = control.compute_forward_simulations_baseline(x, x_des, parameters[0])
    print("baseline computation: ", time.time()-start_time)
    #print("cost: ", cost)

    


    start_time = time.time()
    cost = control.compute_forward_simulations(x, x_des, parameters[0])
    print("non compiled jax single: ", time.time()-start_time)
    #print("cost: ", cost)

    start_time = time.time()
    jit_fd = jax.jit(control.compute_forward_simulations)
    #print("compilation jax: ", time.time()-start_time)
    
    start_time = time.time() 
    cost = jit_fd(x, x_des, parameters[0])
    #print("compiled jax: ", time.time()-start_time)
    #print("cost: ", cost)
    
    start_time = time.time()
    cost = jit_fd(x, x_des, parameters[0])
    print("compiled jax single: ", time.time()-start_time)
    #print("cost: ", cost)


    # parallel computation test ------------------------------------

    threads = 100000
    xs = jnp.tile(x, (threads,1)).reshape(threads,6)
    xs_des = jnp.tile(x_des, (threads,1)).reshape(threads,6)

    
    #print("xs", xs)

    key = random.PRNGKey(42)
    parameters_map = random.randint(key,(4*threads,), minval=-40, maxval=40 )/20.0
    parameters_map = parameters_map.reshape(threads,4)

    #print("parameters", parameters_map)

    
    v_fd = jax.vmap(control.compute_forward_simulations, in_axes=(0,0,0), out_axes=0)
    start_time = time.time()
    cost = v_fd(xs, xs_des, parameters_map)
    print("non compiled VMAP jax: ", time.time()-start_time)
    #print("costs_out", cost)
    #print("minimum cost", np.nanmin(cost))
    min_cost_index = np.nanargmin(cost)
    #print("minimum cost index", min_cost_index)
    #print("best parameters", parameters_map[min_cost_index])
    
    start_time = time.time()
    jit_v_fd = jax.jit(v_fd)
    #print("parallel compilation jax: ", time.time()-start_time)

    start_time = time.time()
    costs = jit_v_fd(xs, xs_des, parameters_map)
    #print("parallel compiled jax: ", time.time()-start_time)
    #print("costs", costs)

    start_time = time.time()
    costs = jit_v_fd(xs, xs_des, parameters_map)
    #print("costs_out", costs)
    #print("minimum cost", np.nanmin(cost))
    print("parallel VMAP jax: ", time.time()-start_time)
    
    
    
 
    '''start_time = time.time()
    for i in range(0,threads):
        parameters = np.random.rand(10,1).reshape(-1,10)
        cost = jit_fd(x, x_des, parameters)
        #cost = control.compute_forward_simulation(x, x_des)
        #print(cost)
    print("non parallel compiled jax: ", time.time()-start_time)'''

    
    
    

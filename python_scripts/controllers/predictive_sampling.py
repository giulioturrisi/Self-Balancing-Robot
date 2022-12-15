import jax
import jax.numpy as jnp
from jax import jit
from jax import random

import numpy as np
import time
import sys
sys.path.append('/home/python_scripts/')

from twip_dynamics_jax import Twip_dynamics_jax



class Sampling_MPC:
    """This is a small class that implements a sampling based control law"""


    def __init__(self, horizon = None, dt = None):
        """
        Args:
            horizon (int): how much to look into the future for optimizing the gains 
            dt (int): desidered sampling time
        """
        self.horizon = horizon
        self.dt = dt
        self.state_dim = 6
        self.control_dim = 2

        self.twip = Twip_dynamics_jax()

        self.Q = jnp.identity(6)

        self.Q.at[0,0].set(0.0)
        self.Q.at[1,1].set(5.0)
        self.Q.at[2,2].set(0.0)
        self.Q.at[3,3].set(2.0)
        self.Q.at[4,4].set(1.0)
        self.Q.at[5,5].set(0.2)

        self.R = jnp.identity(2)*10

        self.state_vec = jnp.zeros(((self.horizon+1), self.state_dim, 1));
        self.control_vec = jnp.zeros(((self.horizon), self.control_dim, 1));

        
    

    def compute_forward_simulation(self, initial_state, state_des, parameters):
        """Calculate rollout

        Args:
            initial_state (np.array): actual state of the robot
        """

        '''print("######### init")
        print("initial_state", initial_state)
        print("state_des", state_des)
        print("parameters prima", parameters)'''
        #parameters = parameters[0]
        #print("parameters dopo", parameters)

        
        

        state = initial_state
        cost = 0
        cost = cost+1
        
        for step in range(0,self.horizon):

            # simulate system

            
            u_0 = jax.numpy.interp(step, jnp.array([self.horizon - 17, self.horizon - 10, self.horizon - 5, self.horizon]), jnp.array([parameters[0], parameters[1], parameters[2], parameters[3]]))
            u_1 = jax.numpy.interp(step, jnp.array([self.horizon - 17, self.horizon - 10, self.horizon - 5, self.horizon]), jnp.array([parameters[4], parameters[5], parameters[6], parameters[7]]))
            

            control = jnp.array([u_0, u_1])
            control = jax.numpy.where(control > 0.5, 0.5, control)
            control = jax.numpy.where(control < -0.5, -0.5, control)
            
            #print("control", control)
            #qdd = self.twip.forward_dynamics(state,control);
            qdd = self.twip.forward_dynamics(state.reshape(self.state_dim,),jnp.array([u_0, u_1])); 
            qdd = qdd[3:6]

            #print("state pre int", state)
            
            # integration
            #self.state_vec.at[step+1].set(self.twip.euler_integration(state, qdd, self.dt).reshape(self.state_dim,1))
            state = self.twip.euler_integration(state, qdd, self.dt).reshape(self.state_dim,1)
            
            #print("state", state)
            cost += (state - state_des.reshape(self.state_dim,1)).T@self.Q@(state - state_des.reshape(self.state_dim,1))
        #print("cost", cost)
        return cost
    
    
    
    
    def compute_control(self, state, state_des):
        """Compute control inputs

        Args:
            state (np.array): actual robot state
            state_des (np.array): desired robot state

        Returns:
            (np.array): optimized control inputs

        """

        return [0,0]

def sum_vector(x: np.ndarray) -> np.ndarray:
    """Assumes `x` is a vector"""
    print("x", x)
    return np.sum(x)



if __name__=="__main__":
    control = Sampling_MPC(dt=0.01, horizon=50)
    threads = 1000
    
    x = jnp.array([0, 0, 0, 0, 1., 0.])
    x_des = jnp.array([0, 0, 0, 0, 1., 0.])
    key = random.PRNGKey(42)
    parameters = random.uniform(key,(10,)).reshape(-1,10)
    
    start_time = time.time()
    control.compute_forward_simulation(x, x_des, parameters[0])
    print("no compiled jax: ", time.time()-start_time)



    start_time = time.time()
    jit_fd = jax.jit(control.compute_forward_simulation)
    print("compilation jax: ", time.time()-start_time)
    start_time = time.time()
    jit_fd(x, x_des, parameters[0])
    print("compiled jax: ", time.time()-start_time)

    start_time = time.time()
    jit_fd(x, x_des, parameters[0])
    print("compiled2 jax: ", time.time()-start_time)


    a = np.arange(20).reshape((4, 5))
    print("a", a)
    jax.vmap(sum_vector)(a)

    print("\n")

    threads = 3000
    xs = jnp.zeros((6*threads,)).reshape(threads,6)
    xs_des = jnp.zeros((6*threads,)).reshape(threads,6)# + x_des.reshape(threads,6)
    

    key = random.PRNGKey(42)
    parameters_map = random.uniform(key,(10*threads,)).reshape(threads,10)

    print("xs", xs.shape)
    print("xs_des", xs_des.shape)
    print("parameters_map", parameters_map.shape)    

    start_time = time.time()
    v_fd = jax.vmap(control.compute_forward_simulation, in_axes=(0,0,0), out_axes=0)
    print("costs_out", v_fd(xs, xs_des, parameters_map))
    print("VMAP jax: ", time.time()-start_time)
    
    
    start_time = time.time()
    jit_v_fd = jax.jit(v_fd)
    print("parallel compilation jax: ", time.time()-start_time)

    start_time = time.time()
    jit_v_fd(xs, xs_des, parameters_map)
    print("parallel compiled jax: ", time.time()-start_time)

    start_time = time.time()
    costs = jit_v_fd(xs, xs_des, parameters_map)
    print("parallel compiled2 jax: ", time.time()-start_time)
    print("costs", costs)
    
    
 
    '''start_time = time.time()
    for i in range(0,threads):
        parameters = np.random.rand(10,1).reshape(-1,10)
        cost = jit_fd(x, x_des, parameters)
        #cost = control.compute_forward_simulation(x, x_des)
        #print(cost)
    print("non parallel compiled jax: ", time.time()-start_time)'''

    
    
    

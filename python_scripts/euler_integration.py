import numpy as np
import casadi as cs
import jax.numpy as jnp

def euler_integration(state, qdd, dt):
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
    x_dot = state[2]
    
    pitch = state[1]
    pitch_dot = state[4]
    
    yaw = state[3]
    yaw_dot = state[5]
    
    x = x + x_dot*dt
    x_dot = x_dot + qdd[0]*dt
    #x = x + x_dot*dt

    pitch = pitch + pitch_dot*dt
    pitch_dot = pitch_dot + qdd[1]*dt
    #pitch = pitch + pitch_dot*dt

    yaw = yaw + yaw_dot*dt
    yaw_dot = yaw_dot + qdd[2]*dt
    #yaw = yaw + yaw_dot*dt

    state = np.array([x, pitch, yaw, x_dot.__float__(), pitch_dot.__float__(), yaw_dot.__float__()])
    state = state.reshape(6,)

    return state

def euler_integration_cs(state, qdd, dt):
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
    x_dot = state[2]
    
    pitch = state[1]
    pitch_dot = state[4]
    
    yaw = state[3]
    yaw_dot = state[5]
    
    x = x + x_dot*dt
    x_dot = x_dot + qdd[0]*dt
    #x = x + x_dot*dt

    pitch = pitch + pitch_dot*dt
    pitch_dot = pitch_dot + qdd[1]*dt
    #pitch = pitch + pitch_dot*dt

    yaw = yaw + yaw_dot*dt
    yaw_dot = yaw_dot + qdd[2]*dt
    #yaw = yaw + yaw_dot*dt

    state = np.array([x, pitch, yaw, x_dot, pitch_dot, yaw_dot])
    state = state.reshape(6,)

    return state


def euler_integration(state, qdd, dt):
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


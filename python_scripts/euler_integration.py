import numpy as np

def euler_integration(state, qdd, dt):
    #x, pitch, yaw, xd, pitch_d, yaw_d

    x = state[0] 
    x_dot = state[2]
    
    pitch = state[1]
    pitch_dot = state[4]
    
    yaw = state[3]
    yaw_dot = state[5]
    
    x_dot = x_dot + qdd[0]*dt
    x = x + x_dot*dt

    pitch_dot = pitch_dot + qdd[1]*dt
    pitch = pitch + pitch_dot*dt

    yaw_dot = yaw_dot + qdd[2]*dt
    yaw = yaw + yaw_dot*dt

    state = np.array([x, pitch, yaw, x_dot, pitch_dot, yaw_dot])
    state = state.reshape(6,)

    return state
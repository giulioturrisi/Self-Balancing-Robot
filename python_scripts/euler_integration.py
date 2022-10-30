import numpy as np

def euler_integration(state, qdd, dt):
    #x, pitch, yaw, xd, pitch_d, yaw_d

    x = state[0] 
    x_d = state[1]
    
    pitch = state[2]
    pitch_d = state[3]
    
    yaw = state[4]
    yaw_d = state[5]
    
    x_dot = x_dot + qdd[0]*dt
    x = x + x_dot*dt

    pitch_dot = pitch_dot + qdd[1]*dt
    pitch = pitch + pitch_dot*dt

    yaw_dot = yaw_dot + qdd[2]*dt
    yaw = yaw + yaw_dot*dt

    state = np.array([x, x_dot, pitch, pitch_dot, yaw, yaw_dot])

    return state
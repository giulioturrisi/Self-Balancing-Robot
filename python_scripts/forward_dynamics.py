import numpy as np
import casadi as cs

#robot parameters
m_b = 1.0 #mass base - kg
m_w = 0.25 #wheel mass - kg
I_pitch = 2.167e-02 #inertia pitch
I_roll = 2.167e-02 #inertia roll
I_yaw = 4.167e-03
I_w1 = 5.000e-03 #inertial wheel axis
I_w2 = 2.575e-03 #inertial wheel vertical axis

r = 0.1 #radius wheels
d = 0.2  #distance between wheels
g = 9.8 #gravity
l = 0.2 #lenght body
c_alpha = -0.001 #viscous friction


def forward_dynamics(state, tau):
    #x, pitch, yaw, xd, pitch_d, yaw_d
    x = state[0] 
    pitch = state[1]
    yaw = state[2]

    x_d = state[3]
    pitch_d = state[4]
    yaw_d = state[5]
    
    tau_l = tau[0]    
    tau_r = tau[1]

    a11 = m_b + 2*m_w + 2*I_w1/(r*r)
    a12 = m_b*l*cs.np.cos(pitch)
    a21 = a12
    a22 = I_roll + m_b*l*l
    a33 = I_pitch + 2*I_w2 + (m_w + I_w1/(r*r))*(d*d/2) - (I_roll - I_yaw - m_b*l*l)*cs.np.sin(pitch)*cs.np.sin(pitch)
    c12 = -m_b*l*pitch_d*cs.np.sin(pitch)
    c13 = -m_b*l*yaw_d*cs.np.sin(pitch)
    c23 = (I_yaw - I_roll - m_b*l*l)*yaw_d*cs.np.sin(pitch)*cs.np.cos(pitch)
    c31 = m_b*l*yaw_d*cs.np.sin(pitch)
    c32 = -(I_yaw - I_roll - m_b*l*l)*yaw_d*cs.np.sin(pitch)*cs.np.cos(pitch)
    c33 = -(I_yaw - I_roll - m_b*l*l)*pitch_d*cs.np.sin(pitch)*cs.np.cos(pitch)
    d11 = 2*c_alpha/(r*r)
    d12 = -2*c_alpha/r
    d21 = d12
    d22 = 2*c_alpha
    d33 = ((d*d)/(2*r*r))*c_alpha

    M = cs.np.matrix([[a11, a12, 0], [a21, a22, 0], [0, 0, a33]])  
    C = cs.np.matrix([[0, c12, c13], [0, 0, c23], [c31, c32, c33]])
    B = cs.np.matrix([[1/r, 1/r], [-1, -1], [-d/(2*r), d/(2*r)]])
    G = cs.np.array([[0, -m_b*l*g*cs.np.sin(pitch), 0]]).T
    D = cs.np.matrix([[d11, d12, 0], [d21, d22, 0], [0, 0, d33]])

    tau = cs.np.array([[tau_l, tau_r]]).T
    qd = cs.np.array([[x_d, pitch_d, yaw_d]]).T
    qdd = cs.inv(M)@(-C@qd - G - D@qd + B@tau) 


    return cs.vertcat(qd,qdd)

def inv_control_matrix():
    B = np.matrix([[1/r, 1/r], [-1, -1], [-d/(2*r), d/(2*r)]])
    return np.linalg.pinv(B)

def compute_feed_forward(pitch,vel):
    #return m_b*l*g*np.sin(pitch)/2.
    return (m_b*l*g*np.sin(pitch) + 2*c_alpha*vel/r)/2.

def compute_angle_from_vel(vel):
    #return -m_b*l*g*np.sin(pitch) -2*c_alpha*vel/r
    return np.arcsin(2*c_alpha*vel/r)/(-m_b*l*g)


def compute_A_matrix(state, tau):
    state_sym = cs.SX.sym("state", 6, 1)
    tau_sym = cs.SX.sym("tau", 2, 1)
    forward_dynamics_f = forward_dynamics(state_sym, tau_sym)
    #fd = cs.Function("fd", [state_sym, tau_sym], [forward_dynamics_f])

    A = cs.jacobian(forward_dynamics_f, state_sym)
    A_f = cs.Function("A", [state_sym, tau_sym], [A])

    return np.array(A_f(state, tau))



def compute_B_matrix(state, tau):
    state_sym = cs.SX.sym("state", 6, 1)
    tau_sym = cs.SX.sym("tau", 2, 1)
    forward_dynamics_f = forward_dynamics(state_sym, tau_sym)
    #fd = cs.Function("fd", [state, tau], [forward_dynamics_f])

    B = cs.jacobian(forward_dynamics_f, tau_sym)
    B_f = cs.Function("B", [state_sym, tau_sym], [B])

    return np.array(B_f(state, tau))


def forward_dynamics_integral(state, reference, tau):
    forward_dynamics_output =  forward_dynamics(state[0:6],tau)
    #print("forward dynamic output", forward_dynamics_output)
    #print("reference - state[6]", reference - state[6])
    #qd = cs.vertcat(forward_dynamics_output[0:3], reference - state[1])
    #print("qd", qd)
    qd = forward_dynamics_output[0:3]
    qdd = cs.vertcat(forward_dynamics_output[3:], state[3] - reference)
    #print("qdd", qdd)
    return cs.vertcat(qd,qdd)


def compute_A_matrix_integral(state, reference, tau):
    state_sym = cs.SX.sym("state", 7, 1)
    tau_sym = cs.SX.sym("tau", 2, 1)
    reference_sym = cs.SX.sym("reference", 1, 1)
    forward_dynamics_f = forward_dynamics_integral(state_sym, reference_sym, tau_sym)
    #fd = cs.Function("fd", [state_sym, tau_sym], [forward_dynamics_f])

    A = cs.jacobian(forward_dynamics_f, state_sym)
    A_f = cs.Function("A", [state_sym, tau_sym], [A])

    return np.array(A_f(state, tau))



def compute_B_matrix_integral(state, reference, tau):
    state_sym = cs.SX.sym("state", 7, 1)
    tau_sym = cs.SX.sym("tau", 2, 1)
    reference_sym = cs.SX.sym("reference", 1, 1)
    forward_dynamics_f = forward_dynamics_integral(state_sym, reference_sym, tau_sym)
    #fd = cs.Function("fd", [state, tau], [forward_dynamics_f])

    B = cs.jacobian(forward_dynamics_f, tau_sym)
    B_f = cs.Function("B", [state_sym, tau_sym], [B])

    return np.array(B_f(state, tau))


'''state = cs.SX.sym("state", 6, 1)
tau = cs.SX.sym("tau", 2, 1)

forward_dynamics_f = forward_dynamics(state, tau)

fd = cs.Function("fd", [state, tau], [forward_dynamics_f])

A = cs.jacobian(forward_dynamics_f, state)

A_f = cs.Function("A", [state, tau], [A])

B = cs.jacobian(forward_dynamics_f, tau)

B_f = cs.Function("B", [state, tau], [B])


print(type(np.array(A_f(np.random.rand(6), np.zeros(2)))))
print(B_f(np.random.rand(6), np.zeros(2)))'''


#fd.generate("fd.c")
#C = cs.Importer("fd.c", "gcc")
#f = cs.external('f',C)



if __name__=="__main__":
    forward_dynamics(np.zeros(6), np.zeros(2))

    '''forward_dynamics_integral(np.zeros(7), 0, np.zeros(2))
    print("compute_A_matrix_integral",compute_A_matrix_integral(np.zeros(7), 0, np.zeros(2)))
    
    
    pitch_des = -0.261799
    print("compute_feed_forward", compute_feed_forward(pitch_des))
    print("compute inv B", inv_control_matrix())'''
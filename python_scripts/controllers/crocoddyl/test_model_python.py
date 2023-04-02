# Display the solution
import numpy as np
import crocoddyl

import sys
sys.path.append('/home/python_scripts/')
from robot_model import Robot_Model

import time

class TwipModelDerived(crocoddyl.ActionModelAbstract):

    def __init__(self):
        crocoddyl.ActionModelAbstract.__init__(self, crocoddyl.StateVector(6), 2, 8)
        self.unone = np.zeros(self.nu)

        self.dt = 0.01

        self.twip = Robot_Model()
        self.costWeights = [0., 1., 0, 1, 2, 1., 10, 10]  # x, pitch, yaw, xdot, pitch_dot, yaw_dot, u_l, u_l

    def calc(self, data, x, u=None):
        if u is None: u = model.unone


        # Defining the equation of motions
        next_state = self.twip.fd(x, u)
        qdd = next_state[3:6]

        data.xnext = self.twip.euler_integration(x, qdd, self.dt).reshape(self.state.nx,1)
        

        # Computing the cost residual and value
        data.r = np.matrix(self.costWeights * np.array([np.asscalar(x[0]), np.asscalar(x[1]), np.asscalar(x[2]), np.asscalar(x[3]), np.asscalar(x[4]), np.asscalar(x[5]), np.asscalar(u[0]), np.asscalar(u[1])])).T
        data.cost = .5 * np.asscalar(sum(np.asarray(data.r)**2))

    def calcDiff(self, data, x, u=None):
        if u is None:
            u = self.unone
        # Cost derivatives
        data.Lx[:] = x * ([np.array([0., 1., 0, 1, 2, 1.])@np.array([0., 1., 0, 1, 2, 1.])] * self.state.nx)
        data.Lu[:] = u * ([np.array([10, 10])@np.array([10, 10])] * self.nu)
        # Dynamic derivatives
        # compute discrete A and B matrices
        A_step = self.twip.A_f(x, u)
        A_step = np.array(A_step)
        B_step = self.twip.B_f(x, u)
        B_step = np.array(B_step)

        A_step = A_step*self.dt + np.identity(self.state.nx)
        B_step = B_step*self.dt
        data.Fx = A_step
        data.Fu = B_step





# Creating the DAM for the twip
model = TwipModelDerived()
twipData = model.createData()

model.r = [
    0.,  # state weight
    1,
    0,
    1,
    2,
    1.,
    10,
    10
]

# Creating the shooting problem
x0 = np.array([0., 1, 0., 0., 0., 0.])
T = 100

start_time = time.time()
problem = crocoddyl.ShootingProblem(x0, [model] * T, model)

# Solving it using FDDP
#ddp = crocoddyl.SolverFDDP(problem)
ddp = crocoddyl.SolverDDP(problem)
ddp.setCallbacks([crocoddyl.CallbackLogger(), crocoddyl.CallbackVerbose()])

ddp.solve()


print("comp. time: ", time.time()-start_time)


print("ddp.K",ddp.K[-1]) 
crocoddyl.plotOCSolution(ddp.xs, ddp.us, figIndex=1)
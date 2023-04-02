# Display the solution
import numpy as np
import crocoddyl

import sys
sys.path.append('/home/python_scripts/')
from robot_model import Robot_Model

import time

class DifferentialActionModelTwip(crocoddyl.DifferentialActionModelAbstract):

    def __init__(self):
        crocoddyl.DifferentialActionModelAbstract.__init__(self, crocoddyl.StateVector(6), 2, 8)  # nu = 1; nr = 6
        self.unone = np.zeros(self.nu)

        self.twip = Robot_Model()
        self.costWeights = [0., 1., 0, 1, 2, 1., 10, 10]  # x, pitch, yaw, xdot, pitch_dot, yaw_dot, u_l, u_l

    def calc(self, data, x, u=None):
        if u is None: u = model.unone


        # Defining the equation of motions
        qdd = self.twip.fd(x, u)
        data.xout = np.matrix([qdd[3].__float__(), qdd[4].__float__(), qdd[5].__float__()]).T

        # Computing the cost residual and value
        data.r = np.matrix(self.costWeights * np.array([np.asscalar(x[0]), np.asscalar(x[1]), np.asscalar(x[2]), np.asscalar(x[3]), np.asscalar(x[4]), np.asscalar(x[5]), np.asscalar(u[0]), np.asscalar(u[1])])).T
        data.cost = .5 * np.asscalar(sum(np.asarray(data.r)**2))

    def calcDiff(self, data, x, u=None):
        # Advance user might implement the derivatives
        pass


# Creating the DAM for the twip
twipDAM = DifferentialActionModelTwip()
twipData = twipDAM.createData()
twipDAM = model = DifferentialActionModelTwip()

# Using NumDiff for computing the derivatives. We specify the
# withGaussApprox=True to have approximation of the Hessian based on the
# Jacobian of the cost residuals.
twipND = crocoddyl.DifferentialActionModelNumDiff(twipDAM, True)

# Getting the IAM using the simpletic Euler rule
timeStep = 0.01
twipIAM = crocoddyl.IntegratedActionModelEuler(twipND, timeStep)

# Creating the shooting problem
x0 = np.array([0., 1, 0., 0., 0., 0.])
T = 100

terminalTwip = DifferentialActionModelTwip()
terminalTwipDAM = crocoddyl.DifferentialActionModelNumDiff(terminalTwip, True)
terminalTwipIAM = crocoddyl.IntegratedActionModelEuler(terminalTwipDAM)

terminalTwip.costWeights[0] = 0
terminalTwip.costWeights[1] = 100
terminalTwip.costWeights[2] = 0
terminalTwip.costWeights[3] = 1
terminalTwip.costWeights[4] = 1
terminalTwip.costWeights[5] = 1
terminalTwip.costWeights[6] = 10
terminalTwip.costWeights[7] = 10

start_time = time.time()
problem = crocoddyl.ShootingProblem(x0, [twipIAM] * T, terminalTwipIAM)

# Solving it using FDDP
#ddp = crocoddyl.SolverFDDP(problem)
ddp = crocoddyl.SolverDDP(problem)
ddp.setCallbacks([crocoddyl.CallbackVerbose()])
ddp.solve([], [], 10)


print("comp. time: ", time.time()-start_time)


print("ddp.K",ddp.K[-1]) 
crocoddyl.plotOCSolution(ddp.xs, ddp.us, figIndex=1)
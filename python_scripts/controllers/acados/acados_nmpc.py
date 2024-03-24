from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from acados_model import export_robot_model
import numpy as np
import scipy.linalg

import time

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path + '/./../../')
from robot_model import Robot_Model


class NMPC:
    def __init__(self, horizon, dt):
        self.horizon = horizon  # Define the number of discretization steps
        self.dt = dt

        self.T_horizon = self.horizon*self.dt

        self.twip = Robot_Model()

        self.ocp = self.create_ocp_solver_description()
        self.acados_ocp_solver = AcadosOcpSolver(
            self.ocp, json_file="acados_nmpc_" + self.ocp.model.name + ".json"
        )

        self.state_dim = self.ocp.model.x.size()[0]
        self.control_dim = self.ocp.model.u.size()[0]

    def create_ocp_solver_description(self,) -> AcadosOcp:
        # create ocp object to formulate the OCP
        ocp = AcadosOcp()

        model = export_robot_model()
        ocp.model = model
        nx = model.x.size()[0]
        nu = model.u.size()[0]
        ny = nx + nu

        # set dimensions
        ocp.dims.N = self.horizon

        # set cost
        Q_mat = 2 * np.diag([0, 5, 0.0, 2, 1, 0.2])  # [x,y,x_d,y_d,th,th_d]
        R_mat = 2 * 5 * np.diag([1, 1])

        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.cost_type_e = "LINEAR_LS"

        ny = nx + nu
        ny_e = nx

        ocp.cost.W_e = Q_mat
        ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)

        ocp.cost.Vx = np.zeros((ny, nx))
        ocp.cost.Vx[:nx, :nx] = np.eye(nx)

        Vu = np.zeros((ny, nu))
        Vu[nx : nx + nu, 0:nu] = np.eye(nu)
        ocp.cost.Vu = Vu

        ocp.cost.Vx_e = np.eye(nx)

        ocp.cost.yref = np.zeros((ny,))
        ocp.cost.yref_e = np.zeros((ny_e,))

        # set constraints
        tau_max = 0.5 
        ocp.constraints.lbu = np.array([-tau_max, -tau_max])
        ocp.constraints.ubu = np.array([+tau_max, +tau_max])
        ocp.constraints.idxbu = np.array([0,1])

        X0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        ocp.constraints.x0 = X0

        # set options
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # 'GAUSS_NEWTON', 'EXACT'
        ocp.solver_options.integrator_type = "IRK"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"  # SQP_RTI, SQP
        ocp.solver_options.nlp_solver_max_iter = 400
        # ocp.solver_options.levenberg_marquardt = 1e-2

        # set prediction horizon
        ocp.solver_options.tf = self.T_horizon

        return ocp

    def compute_control(self, state, state_des):
        state_des[1] = self.twip.compute_angle_from_vel(state_des[3])

        # initialize solver
        for stage in range(self.horizon + 1):
            self.acados_ocp_solver.set(stage, "x", 0.0 * np.ones((self.state_dim,)))
        for stage in range(self.horizon):
            self.acados_ocp_solver.set(stage, "u", np.zeros((self.control_dim,)))

        for j in range(self.horizon):
            yref = np.array([state_des[0], state_des[1], state_des[2], state_des[3], state_des[4], state_des[5], 0, 0])
            self.acados_ocp_solver.set(j, "yref", yref)
        yref_N = np.array(state_des)
        self.acados_ocp_solver.set(self.horizon, "yref", yref_N)


        # set initial state constraint
        self.acados_ocp_solver.set(0, "lbx", state)
        self.acados_ocp_solver.set(0, "ubx", state)


        # solve ocp
        status = self.acados_ocp_solver.solve()

        control = self.acados_ocp_solver.get(0, "u")



        return control

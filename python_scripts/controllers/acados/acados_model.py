from acados_template import AcadosModel
from casadi import SX, vertcat

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path + '/./../../')
from robot_model import Robot_Model

# Reference for model equations:
# http://users.isr.ist.utl.pt/~jag/publications/08-JETC-RCarona-vcontrol.pdf

def export_robot_model() -> AcadosModel:
    model_name = "twip"

    # set up states & controls
    x = SX.sym("x")
    pitch = SX.sym("pitch")
    yaw = SX.sym("yaw")

    x_d = SX.sym("x_d")
    pitch_d = SX.sym("pitch_d")
    yaw_d = SX.sym("yaw_d")

    x = vertcat(x, pitch, yaw, x_d, pitch_d, yaw_d)

    u_l = SX.sym("u_l")
    u_r = SX.sym("u_r")
    u = vertcat(u_l, u_r)

    # xdot
    x_dot = SX.sym("x_dot")
    pitch_dot = SX.sym("pitch_dot")
    yaw_dot = SX.sym("yaw_dot")
    x_ddot = SX.sym("x_ddot")
    pitch_ddot = SX.sym("pitch_ddot")
    yaw_ddot = SX.sym("yaw_ddot")


    xdot = vertcat(x_dot, pitch_dot, yaw_dot, x_ddot, pitch_ddot, yaw_ddot)

    # algebraic variables
    # z = None

    # parameters
    p = []

    # dynamics
    #f_expl = vertcat(v * cos(theta), v * sin(theta), F, theta_d, T)
    twip = Robot_Model()
    f_expl = twip.fd(x, u)

    print("f_expl", f_expl)

    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    # model.z = z
    model.p = p
    model.name = model_name

    return model
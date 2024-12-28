import rclpy # type: ignore 
from rclpy.node import Node # type: ignore

from std_msgs.msg import String, Bool, Float64, Float64MultiArray # type: ignore
from tf2_msgs.msg import TFMessage # type: ignore
from geometry_msgs.msg import PoseStamped, Twist, Vector3 # type: ignore

import copy
import matplotlib.pyplot as plt # type: ignore
import math
import time
import numpy as np # type: ignore
import sys
np.set_printoptions(threshold=sys.maxsize)

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

sys.path.append(dir_path + '/../../../../simulations/mujoco_simulation')
sys.path.append(dir_path + '/../../../../python_scripts')

from mujoco_simulation import MujocoSimulation

class Wrapper_Mujoco_simulation(Node):
    def __init__(self,):
        super().__init__('Mujoco_Simulation')

        self.sim = MujocoSimulation()

        self.subscription_command = self.create_subscription(Vector3, 'torques', self.cmd_callback, 1)
        self.old_commanded_torque = np.zeros(2)

        self.publisher_state = self.create_publisher(Float64MultiArray, 'state', 1)
        self.create_timer(0.002, self.publish_state)


    def cmd_callback(self, msg):
        commanded_torque_fl = msg.x
        commanded_torque_fr = msg.y
        commanded_torque = np.array([commanded_torque_fl, commanded_torque_fr])
        
        self.old_commanded_torque = commanded_torque
       


    def publish_state(self,):

        state_temp = self.sim.step(self.old_commanded_torque[0], self.old_commanded_torque[1])
        
        state_robot = Float64MultiArray()
        state_robot.data = state_temp.tolist()
        self.publisher_state.publish(state_robot)

        self.sim.render()

        




def main(args=None):
    rclpy.init(args=args)

    sim_node = Wrapper_Mujoco_simulation()

    rclpy.spin(sim_node)
    sim_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
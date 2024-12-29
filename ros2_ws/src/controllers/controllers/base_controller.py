import rclpy # type: ignore 
from rclpy.node import Node # type: ignore

from std_msgs.msg import String, Bool, Float64, Float64MultiArray # type: ignore
from tf2_msgs.msg import TFMessage # type: ignore
from geometry_msgs.msg import PoseStamped, Twist, Vector3 # type: ignore
import tf_transformations # type: ignore

import copy
import matplotlib.pyplot as plt # type: ignore
import math
import time
import sys
import numpy as np # type: ignore
np.set_printoptions(threshold=sys.maxsize)


class Base_Controller(Node):
    def __init__(self, name):
        super().__init__(name)

        self.state_arrived = False
        self.dt = 0.01

        self.x_world = 0
        self.y_world = 0

        self.state_robot = np.zeros(6)
        self.old_state_robot = np.zeros(6)
        self.state_d = np.zeros(6)
        
        
        self.subscription_tf = self.create_subscription(Float64MultiArray,'state',self.state_callback,1)
        self.subscription_tf = self.create_subscription(TFMessage,'tf',self.tf_callback,1)
        self.subscription_cmd_vel = self.create_subscription(Twist,"cmd_vel", self.getVel_callback, 1);

        '''self.publisher_command = self.create_publisher(Float64MultiArray,"torques", 1);
        self.publisher_motor_left = self.create_publisher(Float64,"leftMotor", 1);
        self.publisher_motor_right = self.create_publisher(Float64,"rightMotor", 1);'''
        self.publisher_command = self.create_publisher(Vector3,"torques", 1);


 
        # Sincronization with CoppeliaSim  ---------------------------------------
        self.enableSyncMode = Bool()
        self.enableSyncMode.data = False # Put this to True if you want to synchronize with CoppeliaSim
        self.publisher_enableSyncMode =self.create_publisher(Bool,"enableSyncMode", 1)
        self.publisher_enableSyncMode.publish(self.enableSyncMode)


        self.triggerNextStep = Bool()
        self.triggerNextStep.data = True
        self.publisher_triggerNextStep = self.create_publisher(Bool,"triggerNextStep", 1)

        self.simStep_done = True
        self.subscription_simStep = self.create_subscription(Bool,'simulationStepDone',self.simStep_callback,1)


    def publish_command(self, torques):

        tau_l = torques[0]
        tau_r = torques[1]

        #commanded_torques = Float64MultiArray();
        #commanded_torques.data = [tau_l, tau_r]
        #self.publisher_command.publish(commanded_torques);

        commanded_torques = Vector3()
        commanded_torques.x = float(tau_l)
        commanded_torques.y = float(tau_r)
        self.publisher_command.publish(commanded_torques)



    def state_callback(self, msg):
        self.state_robot = np.array(msg.data)

        self.state_arrived = True
        self.simStep_done = True



    def tf_callback(self, msg):
        quaternion = [float(msg.transforms[0].transform.rotation.x), float(msg.transforms[0].transform.rotation.y), 
                      float(msg.transforms[0].transform.rotation.z), float(msg.transforms[0].transform.rotation.w)]
        euler = tf_transformations.euler_from_quaternion(quaternion)

        x_d_world = (msg.transforms[0].transform.translation.x - self.x_world)/self.dt
        y_d_world = (msg.transforms[0].transform.translation.y - self.y_world)/self.dt
        self.x_world = msg.transforms[0].transform.translation.x
        self.y_world = msg.transforms[0].transform.translation.y

        self.old_state_robot = copy.deepcopy(self.state_robot)

        self.state_robot[3] = math.cos(euler[2])*x_d_world + math.sin(euler[2])*y_d_world
        self.state_robot[4] = (euler[1] - self.state_robot[1])/self.dt #derivative pitch
        self.state_robot[5] = (euler[2] - self.state_robot[2])/self.dt #derivative yaw

        self.state_robot[0] = msg.transforms[0].transform.translation.x
        self.state_robot[1] = euler[1] #pitch
        self.state_robot[2] = euler[2] #yaw
        
        self.state_arrived = True


    def simStep_callback(self,msg):
        self.simStep_done = True

    def getVel_callback(self, msg):
        self.state_d[3] = msg.linear.x
        self.state_d[5] = msg.angular.z


    def triggerNextStep_Sim(self,):
        self.simStep_done = False
        self.publisher_triggerNextStep.publish(self.triggerNextStep)



def main(args=None):
    rclpy.init(args=args)

    controller_node = Base_Controller()

    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
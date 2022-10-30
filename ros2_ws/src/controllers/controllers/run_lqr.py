import rclpy # type: ignore 
from rclpy.node import Node # type: ignore

from std_msgs.msg import String, Bool, Float64, Float64MultiArray # type: ignore
from tf2_msgs.msg import TFMessage # type: ignore
import tf_transformations # type: ignore

import copy
import matplotlib.pyplot as plt # type: ignore
import math
import time
import sys
import numpy as np # type: ignore
np.set_printoptions(threshold=sys.maxsize)

sys.path.append('/home/python_scripts/controllers')
from lqr import LQR 



class Controller(Node):
    def __init__(self):
        super().__init__('LQR')
        self.ref_x_d = 0
        self.ref_yaw_d = 0
        self.vel_ready = False;
        self.state_arrived = False
        self.dt = 0.01

        self.state_robot = np.zeros(6)

        self.create_timer(self.dt, self.controller_callback)
        
        #self.subscription_tf = self.create_subscription(Float64MultiArray,'state',self.state_callback,1)
        self.subscription_tf = self.create_subscription(TFMessage,'tf',self.tf_callback,1)
        #self.subscription_path = self.create_subscription(Path,'path',self.getPath_callback,1)

        self.publisher_command = self.create_publisher(Float64MultiArray,"torques", 1);
        self.publisher_motor_left = self.create_publisher(Float64,"leftMotor", 1);
        self.publisher_motor_right = self.create_publisher(Float64,"rightMotor", 1);

        #self.horizon = 30
        self.controller = LQR()
        #self.controller = Casadi_nmpc(self.horizon,[],[], 0.01)



        # Sincronization with simulation ---------------------------------------
        self.enableSyncMode = Bool();
        self.enableSyncMode.data = True;
        self.publisher_enableSyncMode =self.create_publisher(Bool,"enableSyncMode", 1);
        self.publisher_enableSyncMode.publish(self.enableSyncMode)


        self.triggerNextStep = Bool();
        self.triggerNextStep.data = True;
        self.publisher_triggerNextStep = self.create_publisher(Bool,"triggerNextStep", 1);

        self.simStep_done = True
        self.subscription_simStep = self.create_subscription(Bool,'simulationStepDone',self.simStep_callback,1)



    def controller_callback(self):
        if(self.simStep_done):
            print("###############")
            print("state robot: ", self.state_robot)
            start_time = time.time()

            state_d = np.zeros(6)
            torques = self.controller.compute_control(self.state_robot, state_d)

            print("torques", torques)

            #self.controller.initialize_casadi()
            #tau_l, tau_r = self.controller.compute_control(self.state_robot, self.ref_x_d, self.ref_yaw_d)
            tau_l = torques[0]
            tau_r = torques[1]

            print("control time: ", time.time()-start_time)

            commanded_torques = Float64MultiArray();
            #commanded_torques.push_back(0.0); #left
            #commanded_torques.push_back(0.0); #right
            commanded_torques.data = [tau_l, tau_r]

            #self.publisher_command.publish(commanded_torques);

            commanded_torque_left = Float64();
            commanded_torque_left.data = tau_l
            self.publisher_motor_left.publish(commanded_torque_left)


            commanded_torque_right = Float64();
            commanded_torque_right.data = tau_r
            self.publisher_motor_right.publish(commanded_torque_right)
            
        #else:
            # Zero control inputs ---------------------------------------
            #commanded_vel = Twist();
            #commanded_vel.linear.x = 0.0;
            #commanded_vel.angular.z = 0.0;
            #self.publisher_command.publish(commanded_vel);


        # Trigger next step Simulation ---------------------------------------
        self.simStep_done = False
        self.publisher_triggerNextStep.publish(self.triggerNextStep)
        

    def state_callback(self, msg):
        #if(msg.transforms[0].child_frame_id == "base_footprint"):
        #    quaternion = [float(msg.transforms[0].transform.rotation.x), float(msg.transforms[0].transform.rotation.y), 
        #        float(msg.transforms[0].transform.rotation.z), float(msg.transforms[0].transform.rotation.w)]
        #    euler = tf_transformations.euler_from_quaternion(quaternion)
            
        self.state_robot[0] = msg.x
        self.state_robot[1] = msg.x_d
        self.state_robot[2] = msg.pitch
        self.state_robot[3] = msg.pitch_d
        self.state_robot[4] = msg.yaw
        self.state_robot[5] = msg.yaw_d

        self.state_arrived = True

    def tf_callback(self, msg):
        #if(msg.transforms[0].child_frame_id == "base_footprint"):
        quaternion = [float(msg.transforms[0].transform.rotation.x), float(msg.transforms[0].transform.rotation.y), 
                      float(msg.transforms[0].transform.rotation.z), float(msg.transforms[0].transform.rotation.w)]
        euler = tf_transformations.euler_from_quaternion(quaternion)


        self.state_robot[3] = (msg.transforms[0].transform.translation.x - self.state_robot[0])/self.dt
        self.state_robot[4] = (euler[1] - self.state_robot[1])/self.dt #derivative pitch
        self.state_robot[5] = (euler[2] - self.state_robot[2])/self.dt #derivative yaw

        self.state_robot[0] = msg.transforms[0].transform.translation.x
        self.state_robot[1] = euler[1] #pitch
        self.state_robot[2] = euler[2] #yaw
        
        
        print("pitch_robot: ", self.state_robot[1])


        self.state_arrived = True


    def simStep_callback(self,msg):
        self.simStep_done = True

    def getVel_callback(self, msg):
        self.ref_x_d_des = msg.ref_x_d_des
        self.ref_yaw_d_des = msg.ref_yaw_d_des

        #self.vel_ready = True;
        print("new vel received")



def main(args=None):
    rclpy.init(args=args)
    print("###### Controller started ######")

    controller_node = Controller()

    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

from geometry_msgs.msg import PoseStamped, Twist, Vector3 # type: ignore
from geometry_msgs.msg import TwistStamped

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))
import sys
sys.path.append(dir_path + '/./../../../../python_scripts/state_estimators')
sys.path.append(dir_path)

from extended_kalman_filter import Extended_Kalman_Filter 

import numpy as np

class StatePublisher(Node):

    def __init__(self):
        #rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.subscription_imu = self.create_subscription(Vector3, 'imu', self.imu_callback,1)
        self.subscription_control = self.create_subscription(Vector3, 'torques', self.control_callback,1)
        
        dt = 0.005
        self.create_timer(dt, self.tf_callback)

        self.last_time = 0.0

        self.tau_l = 0.0
        self.tau_r = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.EKF = Extended_Kalman_Filter(V = 1, W = 1, P = 1, dt = dt)
        self.state_robot_filtered = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


    def tf_callback(self):
        
        measurements = [0.0, self.pitch, self.yaw, 0.0, 0.0, 0.0] 
        controls = [self.tau_l, self.tau_r]
        self.state_robot_filtered = self.EKF.compute_filter(self.state_robot_filtered, controls, measurements)

        print("ground truth pitch: ", self.pitch)
        print("filtered pitch: ", self.state_robot_filtered[1])
        print("ground truth yaw: ", self.yaw)
        print("filtered yaw: ", self.state_robot_filtered[2])
        print("##########")

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'body__32__'
        joint_state = JointState()
        # update joint_state
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['Revolute_joint1__33__', 'Revolute_joint2__35__']
        joint_state.position = [0.0, 0.0]

        # update transform
        odom_trans.header.stamp = now.to_msg()
        odom_trans.transform.translation.x = 0.0 
        odom_trans.transform.translation.y = 0.0 
        odom_trans.transform.translation.z = sin(pi/2. - self.pitch)*0.25
        odom_trans.transform.rotation = \
            euler_to_quaternion(self.roll, self.pitch, self.yaw) # roll,pitch,yaw #self.odom_theta


        # send the joint state and transform
        self.joint_pub.publish(joint_state)
        self.broadcaster.sendTransform(odom_trans)

       

    def odometry_callback(self, msg):
        print("odometry received")

    

    def imu_callback(self, msg):
        print("imu received")
        self.roll = msg.x
        self.pitch = msg.y
        self.yaw = msg.z

    def control_callback(self, msg):
        print("control received")
        self.tau_l = msg.x
        self.tau_r = msg.y


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    rclpy.init()
    node = StatePublisher()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
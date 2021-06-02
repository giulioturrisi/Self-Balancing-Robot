import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
#from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Imu
from time import sleep

#from tf2 import utils
#import tf2_ros

import smbus
import math


import signal
import sys


from threading import Thread
#TO RUN
#ros2 run imu publisher
#CONNECTION
#red (VDD) brown(GROUND) yellow(SCL - PIN GPIO 1/ID_SC) orange(SDA - PIN GPIO 0/ID_SD)
#
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu', 1)
        self.initialize_imu()
        self.dt = 0.001  # seconds
        self.timer = self.create_timer(self.dt, self.imu_callback)


    def initialize_imu(self,):
        power_mgmt_1 = 0x6b
        power_mgmt_2 = 0x6c
        self.bus = smbus.SMBus(0) # or bus = smbus.SMBus(1) for Revision 2 boards
        self.address = 0x68 # This is the address value read via the i2cdetect command
        
        # Now wake the 6050 up as it starts in sleep mode
        self.bus.write_byte_data(self.address, power_mgmt_1, 0)

        self.gyroAngleX = 0
        self.gyroAngleY = 0
        self.gyroAngleZ = 0
        self.last_complementary_x = 0
        self.actual_complementary_x = 0
        self.anglex = 0
        self.angley = 0

        self.alpha = 0.9

        print("Ok initialization")

    def read_byte(self,adr):
        return self.bus.read_byte_data(self.address, adr)
    
    def read_word(self,adr):
        high = self.bus.read_byte_data(self.address, adr)
        low = self.bus.read_byte_data(self.address, adr+1)
        val = (high << 8) + low
        return val
    
    def read_word_2c(self,adr):
        val = self.read_word(adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val
    
    def dist(self,a,b):
        return math.sqrt((a*a)+(b*b))
    
    def get_y_rotation(self,x,y,z):
        radians = math.atan2(x, self.dist(y,z))
        #return -math.degrees(radians)
        return radians

    def get_x_rotation(self,x,y,z):
        radians = math.atan2(y, self.dist(x,z))
        #return math.degrees(radians)
        return radians

    def imu_callback(self):

        #ACCELEROMETER

        accel_xout = self.read_word_2c(0x3b)
        accel_yout = self.read_word_2c(0x3d)
        accel_zout = self.read_word_2c(0x3f)
        
        accel_xout_scaled = accel_xout / 16384.0
        accel_yout_scaled = accel_yout / 16384.0
        accel_zout_scaled = accel_zout / 16384.0
        
        
        #print("x rotation acc: " , self.get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))
        #print("y rotation acc: " , self.get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))
        
        self.anglex = self.get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
        self.angley = self.get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)

        #GYRO
        
        gyro_xout = self.read_word_2c(0x43)/131
        gyro_yout = self.read_word_2c(0x45)/131
        gyro_zout = self.read_word_2c(0x47)/131
        
        #self.gyroAngleX = self.anglex + gyro_xout*self.dt*math.pi/180.
        self.gyroAngleX = self.actual_complementary_x + gyro_xout*self.dt*math.pi/180.
        self.gyroAngleY = self.angley + (gyro_yout - 1.70)*self.dt*math.pi/180.
        self.gyroAngleZ = self.gyroAngleZ + (gyro_zout- 1.54)*self.dt

        #print("x rotation gyro: " , gyro_xout)
        #print("y rotation gyro: " , gyro_yout)

        #FILTER        
        
        self.last_complementary_x = self.actual_complementary_x
        complementary_x = self.anglex*(1-self.alpha) + (self.gyroAngleX)*(self.alpha)
        complementary_y = self.angley*(1-self.alpha) + (self.gyroAngleY)*(self.alpha)

        self.actual_complementary_x = complementary_x

        
        #print("xdot rotation FILTERED: " , (complementary_x - self.last_complementary_x)/0.01)

        self.gyroAngleX = complementary_x
        self.gyroAngleY = complementary_y
        
        #print("#######################")
        
        #print("x rotation FILTERED: " , complementary_x )
        #print("y rotation FILTERED: " , self.angley)
        #print("z rotation FILTERED: " , self.gyroAngleZ*math.pi/180.)

     

        imu_msg = Imu()
        imu_msg.header.stamp = Node.get_clock(self).now().to_msg()
        imu_msg.linear_acceleration.x = accel_xout_scaled
        imu_msg.linear_acceleration.y = accel_yout_scaled
        imu_msg.linear_acceleration.z = accel_zout_scaled


        #imu_msg.angular_velocity.x = gyro_xout
        imu_msg.angular_velocity.x = (complementary_x - self.last_complementary_x)/self.dt
        imu_msg.angular_velocity.y = gyro_yout
        imu_msg.angular_velocity.z = gyro_zout

        #imu_msg.orientation.x = self.anglex
        #imu_msg.orientation.y = self.angley
        imu_msg.orientation.x = complementary_x
        imu_msg.orientation.y = complementary_y
        imu_msg.orientation.z = self.gyroAngleZ*math.pi/180.
        
        #TO DO - use quaternion field, but need ros 2 foxy
        
        
        self.publisher_.publish(imu_msg)
        
        
   
        
def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        rclpy.shutdown()


def main(args=None):
    #signal.signal(signal.SIGINT, signal_handler)

    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    
    
    print("pulisco")
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

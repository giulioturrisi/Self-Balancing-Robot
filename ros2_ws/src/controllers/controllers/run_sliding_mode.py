import rclpy # type: ignore 

import time
import sys
import numpy as np # type: ignore
np.set_printoptions(threshold=sys.maxsize)


import os 
dir_path = os.path.dirname(os.path.realpath(__file__))
import sys
sys.path.append(dir_path + '/./../../../../python_scripts/controllers')
sys.path.append(dir_path)

from sliding_mode import Sliding_Mode 
from base_controller import Base_Controller



class Controller(Base_Controller):
    def __init__(self):
        super().__init__('SlidingMode')

        self.create_timer(self.dt, self.controller_callback)
        
        self.k_s = 100
        self.k_x_dot = 0.3
        self.k_pitch = 100
        self.k_pitch_dot = 15
        self.k_yaw_dot = 0.
        self.controller = Sliding_Mode(self.k_s, self.k_x_dot, self.k_pitch, self.k_pitch_dot, self.k_yaw_dot)



    def controller_callback(self):
        if(self.enableSyncMode.data == False or self.simStep_done):
            print("###############")
            print("state robot: ", self.state_robot)
            start_time = time.time()

            torques = self.controller.compute_control(self.state_robot, self.state_d)
            
            print("control time: ", time.time()-start_time)

            self.publish_command(torques)


        # Trigger next step Simulation ---------------------------------------
        self.triggerNextStep_Sim()





def main(args=None):
    rclpy.init(args=args)
    print("###### Controller started ######")

    controller_node = Controller()

    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
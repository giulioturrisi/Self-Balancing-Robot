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

from adaptive_lqr import Adaptive_LQR 
from base_controller import Base_Controller



class Controller(Base_Controller):
    def __init__(self):
        super().__init__('Adaptive_LQR')

        self.lin_state = np.zeros(6)
        self.lin_state[1] = 0.0
        self.controller = Adaptive_LQR(dt = self.dt/1., lin_state = self.lin_state)

        self.create_timer(self.dt, self.controller_callback)

        self.old_control = np.zeros(2)

        self.iteration = 0

        self.x_old = np.array([])




    def controller_callback(self):
        if(self.enableSyncMode.data == False or self.simStep_done):
            print("###############")
            print("state robot: ", self.state_robot)
            #print("old robot: ", self.old_state_robot)

            start_time = time.time()
            self.state_d[3] = 1
            self.controller.compute_adaptive_gains(self.old_state_robot, self.state_d, self.state_robot)
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
import rclpy # type: ignore 

import time
import sys
import numpy as np # type: ignore
np.set_printoptions(threshold=sys.maxsize)


import os 
dir_path = os.path.dirname(os.path.realpath(__file__))
import sys
sys.path.append(dir_path + '/./../../../../python_scripts/controllers/acados')
sys.path.append(dir_path)

from acados_nmpc import NMPC 
from base_controller import Base_Controller




class Controller(Base_Controller):
    def __init__(self):
        super().__init__('Acados_NMPC')

        self.create_timer(self.dt, self.controller_callback)
        
        self.horizon = 30
        self.controller = NMPC(horizon=self.horizon, dt=self.dt)
        

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
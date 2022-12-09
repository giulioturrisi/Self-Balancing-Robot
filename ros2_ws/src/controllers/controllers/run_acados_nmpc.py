import rclpy # type: ignore 

import time
import sys
import numpy as np # type: ignore
np.set_printoptions(threshold=sys.maxsize)

sys.path.append('/home/python_scripts/controllers/acados')
from acados_nmpc import NMPC 
sys.path.append('/home/ros2_ws/src/controllers/controllers')
from base_controller import Base_Controller




class Controller(Base_Controller):
    def __init__(self):
        super().__init__('Acados_NMPC')

        self.create_timer(self.dt, self.controller_callback)
        
        self.horizon = 30
        self.controller = NMPC(horizon=self.horizon, dt=self.dt)
        

    def controller_callback(self):
        if(self.simStep_done):
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
import rclpy # type: ignore 

import time
import sys
import numpy as np # type: ignore
np.set_printoptions(threshold=sys.maxsize)

sys.path.append('/home/python_scripts/controllers')
from pid import PID 
sys.path.append('/home/ros2_ws/src/controllers/controllers')
from base_controller import Base_Controller



class Controller(Base_Controller):
    def __init__(self):
        super().__init__('PID')

        self.create_timer(self.dt, self.controller_callback)
        
        self.k_x_d = 0        
        self.k_roll = 500
        self.k_roll_d = 25
        self.k_yaw_d = 0.5
        self.k_i = 0.01

        self.controller = PID(self.k_x_d, self.k_roll, self.k_roll_d, self.k_yaw_d, self.k_i)



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
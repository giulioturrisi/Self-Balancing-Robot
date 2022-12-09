import rclpy # type: ignore 


import time
import sys
import numpy as np # type: ignore
np.set_printoptions(threshold=sys.maxsize)


sys.path.append('/home/python_scripts/controllers')
from ilqr import iLQR 
sys.path.append('/home/ros2_ws/src/controllers/controllers')
from base_controller import Base_Controller


class Controller(Base_Controller):
    def __init__(self):
        super().__init__('iLQR')

        self.lin_state = np.zeros(7)
        self.lin_state[1] = 0.0
        self.controller = iLQR(dt = self.dt/10., lin_state = self.lin_state)

        self.create_timer(self.dt, self.controller_callback)



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
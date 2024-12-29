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

from least_square_lqr import LS_LQR 
from base_controller import Base_Controller



class Controller(Base_Controller):
    def __init__(self):
        super().__init__('LQR_Least_Square')

        self.lin_state = np.zeros(6)
        self.lin_state[1] = 0.0
        self.controller = LS_LQR(dt = self.dt/1., lin_state = self.lin_state)

        self.create_timer(self.dt, self.controller_callback)

        self.old_control = np.zeros(2)

        self.iteration = 0

        self.x_old = np.array([])




    def controller_callback(self):
        if(self.enableSyncMode.data == False or self.simStep_done):
            print("###############")
            print("state robot: ", self.state_robot)
            print("old robot: ", self.old_state_robot)

            start_time = time.time()

            torques = self.controller.compute_control(self.state_robot, self.state_d)

            '''if(self.iteration > 100 ):
                self.controller.recursive_least_square(self.old_state_robot, np.array(self.old_control).reshape(2,), self.state_robot[1:])
                self.old_control = torques'''
            if(self.iteration == 200):
                self.controller.full_least_square(self.x_old, self.u_old, self.y_meas)
            
            elif (self.iteration > 1 and self.iteration < 200):
                if(self.x_old.size == 0):
                    self.x_old = self.old_state_robot.reshape(1,6)
                    self.u_old = np.array(self.old_control).reshape(1,2)
                    self.y_meas = self.state_robot[1:].reshape(1,5)
                else:
                    self.x_old = np.vstack((self.x_old,self.old_state_robot.reshape(1,6)))
                    self.u_old = np.vstack((self.u_old, np.array(self.old_control).reshape(1,2)))
                    self.y_meas = np.vstack((self.y_meas, self.state_robot[1:].reshape(1,5))) 
            
            self.old_control = torques
            
            
            self.iteration += 1
            print("iteration: ", self.iteration)
            print("control time: ", time.time()-start_time)

            # compare error
            next_state = self.controller.twip.forward_dynamics(self.old_state_robot, np.array(self.old_control).reshape(2,))
            qdd = next_state[3:6]
            state_nom = euler_integration.euler_integration(self.old_state_robot, qdd, self.dt)
            error = self.state_robot - state_nom
            lift_state = self.controller.lift_space(self.old_state_robot)
            #pred = self.controller.best_param.T@self.old_state_robot[1:]
            pred = self.controller.best_param.T@lift_state.reshape(self.controller.basis, 1)
            print("error", error[1:])
            #print("pred", pred)
            print("error with pred", error[1:] - pred.T)

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
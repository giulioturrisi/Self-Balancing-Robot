import mujoco
import mujoco.viewer

import numpy as np
from scipy.spatial.transform import Rotation

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))


class MujocoSimulation:
    def __init__(self, ):
        self.scene_file_path = dir_path + "/model/scene.xml"
        self.mjModel = mujoco.MjModel.from_xml_path(str(self.scene_file_path))
        self.mjData = mujoco.MjData(self.mjModel)
        self.viewer = mujoco.viewer.launch_passive(
            self.mjModel, self.mjData, show_left_ui=False, show_right_ui=True
            )
        
        # Reset the simulation
        mujoco.mj_resetDataKeyframe(self.mjModel, self.mjData, 0)

    def step(self, tau_l, tau_r):
        self.mjData.ctrl = np.array([tau_l, tau_r])
        mujoco.mj_step(self.mjModel, self.mjData)

        quat_wxyz = self.mjData.qpos[3:7]
        quat_xyzw = np.roll(quat_wxyz, -1)
        base_ori_euler_xyz_base_frame = Rotation.from_quat(quat_xyzw).as_euler('xyz')
        pitch = base_ori_euler_xyz_base_frame[1]
        yaw = base_ori_euler_xyz_base_frame[2]

        pitch_dot = self.mjData.qvel[1]
        yaw_dot = self.mjData.qvel[2]

        # Calculating horizontal frame velocity
        com_pos = self.mjData.qpos[0:3]  # world frame
        quat_wxyz = self.mjData.qpos[3:7]  # world frame (wxyz) mujoco convention
        quat_xyzw = np.roll(quat_wxyz, -1)  # SciPy convention (xyzw)
        X_B = np.eye(4)
        X_B[0:3, 0:3] = Rotation.from_quat(quat_xyzw).as_matrix()
        X_B[0:3, 3] = com_pos
        R = X_B[0:3, 0:3]
        base_linear_vel_horizontal_frame =  R.T @ self.mjData.qvel[0:3]
        x_dot = base_linear_vel_horizontal_frame[0]


        # The order is [state, state_dot]
        obs = np.array([0.0, pitch, yaw, x_dot, pitch_dot, yaw_dot])

        return obs
        
    
    def render(self):
        self.viewer.sync()


if __name__ == "__main__":
    sim = MujocoSimulation()
    while True:
        obs = sim.step(0.0, 0.0)
        print("obs: ", obs)
        sim.render()
## Overview
This repo contains the code for controlling both a real and a simulated two wheeled inverted pendulum robot via ROS2.

## List of available controllers
1. PID
2. Linear Quadratic Regulator
3. Linear Quadratic Integral Regulator
4. Least Square Linear Quadratic Regulator
5. Adaptive Linear Quadratic Regulator (MIT rule)
6. Sliding Mode
7. RL policy via IsaacGym
8. Nonlinear MPC via Acados
9. Iterative Linear Quadratic Regulator
10. Iterative Linear Quadratic Regulator via Crocoddyl
11. Predictive Sampling 

## List of available state estimators
1. Extended Kalman Filter


## Repository structure
It includes the following folders and subfolders:

1. ```python_scripts```: most of the ROS2 nodes call some classes here
 
2. ```simulations```: scenes used for simulating the robot ([CoppeliaSim](https://www.coppeliarobotics.com/downloads) or [Mujoco](https://github.com/google-deepmind/mujoco))

3. ```ros2_ws```: collection of ROS2 nodes for controlling the robot and some external folders such as ```simExtROS2``` and ```ros2_bubble_rob```(dependencies for CoppeliaSim)



## Build on Linux
1. clone the repo recursively

```sh
git clone --recurse-submodules https://github.com/giulioturrisi/Self-Balancing-Robot.git
```


2. install [miniforge](https://github.com/conda-forge/miniforge/releases) (x86_64) 


3. create an environment using the file in the folder [installation/conda](https://github.com/giulioturrisi/Self-Balancing-Robot/tree/master/installation/conda):

```sh
    conda env create -f mamba_environment.yml
``` 

4. follow the instruction [here](https://robostack.github.io/GettingStarted.html) to install ros-humble


5. add in your .bashrc

```sh
alias twip_env="conda activate twip_env && source your_path_to/Self-Balancing-Robot/ros2_ws/install/setup.bash"
```

6. start your environment and go in ros2_ws
```sh
twip_env
cd your_path_to/Self-Balancing-Robot/ros2_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble
ulimit -s unlimited
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

7. (Optional if you need acados) go inside the [acados](https://github.com/giulioturrisi/Self-Balancing-Robot/tree/master/python_scripts/controllers/acados)/acados folder and press
  
```sh
mkdir build
cd build
cmake -DACADOS_WITH_QPOASES=ON  -DACADOS_WITH_OSQP=ON ..
make install -j4
pip install -e ./../interfaces/acados_template
```
then in your .bashrc, add
```sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/your_path_to/Self-Balancing-Robot/python_scripts/controllers/acados/lib"
export ACADOS_SOURCE_DIR="/your_path_to/Self-Balancing-Robot/python_scripts/controllers/acados"
```

9. (Optional if you want CoppeliaSim) download [CoppeliaSim](https://www.coppeliarobotics.com/) and put in your .bashrc
```
export COPPELIASIM_ROOT_DIR=your_path_to/CoppeliaSim
```


## How to run the simulation
1. on a new terminal 
```sh
ros2 run controllers <control_node>                     
```
where in <control_node> you can choose the type of controller you want. 


2. If using Mujoco 
```sh
ros2 run simulations run_mujoco_simulation                   
```

2. If using Coppeliasim, launch it and run the scene `self_balancing_robot.ttt` in the folder coppeliasim_simulation 
```sh
./coppeliaSim.sh -f your_path_to/Self-Balancing-Robot/coppeliasim_simulation/self_balancing_robot.ttt 
```

3. to command the robot with a joystick
```sh
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```



## Status
Still working in progress, the real robot does not exist yet.

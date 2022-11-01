## Overview
This repo contains the code for controlling both a real and a simulated two wheeled inverted pendulum robot via ROS2. It includes the following folders and subfolders:

1. ```python_scripts```: most of the ROS2 nodes call some classes here
 
2. ```coppeliasim_simulation```: scenes used for simulating the robot

3. ```ros2_ws```: collection of ROS2 nodes for controlling the robot and some external folders such as ```simExtROS2``` and ```ros2_bubble_rob```(dependencies for CoppeliaSim)

 
## Dependencies
1. [ROS2](https://docs.ros.org/en/humble/Installation.html) Humble

2. [CoppeliaSim](https://www.coppeliarobotics.com/downloads) for simulations (not mandatory)




## Build on Linux
1. clone the repo
```sh
git clone --recurse-submodules https://github.com/giulioturrisi/differential_drive.git
```
and extract CoppeliaSim in Self-Balancing-Robot/coppeliasim_simulation

2. build the docker file inside Differential-Drive-Robot/docker_file/integrated_gpu or /nvidia
```sh
docker build -t ros2_humble .
```

4. add alias to start the docker
```sh
cd 
gedit .bashrc
alias twip_humble='xhost + && docker run -it --rm -v /path/to/your_folder/Differential-Drive-Robot:/home/ -v /tmp/.X11-unix:/tmp/.X11-unix:rw -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY  -e QT_X11_NO_MITSHM=1 --gpus all --name ddrive_humble ros2_humble'  (if used /nvidia)
alias twip_humble="xhost + && docker run -it --rm -v /home/giulio/giulio_projects/Differential-Drive-Robot:/home/ -v /tmp/.X11-unix:/tmp/.X11-unix --device=/dev/dri -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY --name ddrive_humble  ros2_humble" (if used /integrated_gpu)
alias twip_humble='xhost + && docker run -it --rm -v /path/to/your_folder/Differential-Drive-Robot:/home/ -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg -v /usr/lib/wsl:/usr/lib/wsl --device=/dev/dxg -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR -e PULSE_SERVER=$PULSE_SERVER -e LD_LIBRARY_PATH=/usr/lib/wsl/lib --name ddrive_humble ros2_humble' (if Windows Linux Subsystem)

alias twip='docker exec -it ddrive_humble bash' (to attach a new terminal to the running docker)
```

5. start docker and build
```sh
twip_humble
cd ros2_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble
ulimit -s unlimited
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```


## How to run the simulation
1. Open Coppeliasim and run the scene `self_balancing_robot.ttt` in the folder coppeliasim_simulation 
```sh
./coppeliaSim.sh -s ../medium_walls_dynamics.ttt -h
```
disable -h flag to run the gui!

2. on a new terminal 
```sh
ros2 run controllers <control_node>                     
```
where in <control_node> you can choose the type of controller you want (for now LQR and Sliding Mode)



Still working in progress. Recently added IsaacGym to learn an RL policy!

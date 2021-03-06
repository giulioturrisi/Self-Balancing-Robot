# ROS2 Interface plugin for CoppeliaSim

Supported ROS2 versions:

 - Foxy Fitzroy

### Compiling

_NOTE:_ the directory containing all files (i.e. package.xml etc) must be called sim_ros2_interface, otherwise build will fail.

1. Install required packages for [libPlugin](https://github.com/CoppeliaRobotics/libPlugin): see libPlugin's [README](external/libPlugin/README.md)
2. Checkout
```
$ git clone --recursive https://github.com/CoppeliaRobotics/simExtROS2Interface.git sim_ros2_interface
```
3. Edit `meta/interfaces.txt` if you need to include more ROS interfaces. You need to specify the fully qualified interface, e.g. geometry_msgs/msg/Twist rather than Twist.
4. Compile
```
$ colcon build --symlink-install
```

Note: if you are reporting a compile error, please use this command to build:
```
VERBOSE=1 MAKEFLAGS=-j1 colcon build --symlink-install --event-handlers console_direct+ --parallel-workers 1
```

Add `--cmake-args -DCMAKE_BUILD_TYPE=Debug` if you are encountering a runtime error (e.g. crash, unexpected behavior, etc...).

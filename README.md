> [!NOTE]  
> Steward OS is no longer being developed by Arbor Robotics and has been archived.

# Steward OS

A friendly tree-planting companion developed by Arbor Robotics at CMU.

[Read the docs.](https://arbor-robotics.github.io/steward/)

[See the video.](https://www.youtube.com/watch?v=MWe7lKiTrrw)

## Building and running

Steward OS is a standard ROS2 workspace, written for ROS2 Humble. That means it's built like any other ROS2 workspace:

```bash
$ . /opt/ros/humble/setup.bash # Source ROS2
$ colcon build --symlink-install # Optionally add "--continue-on-error" in case e.g. you haven't installed the ZED SDK.
$ . install/setup.bash # Source this workspace
$ ros2 launch launch/sim.launch.py # Example launch files are found in the "launch/" directory.
```

FROM ros:foxy-ros1-bridge-focal

RUN apt update

# Link to allow sourcing
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Alias for sourcing
# sr1 -> source ros 1, sr2 -> source ros 2
RUN echo "alias sr1='source /opt/ros/noetic/setup.bash'" >> ~/.bashrc
RUN echo "alias sr2='source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# Example building ROS 1 workspace in dockerfile
RUN cd ~/ros1_ws; source /opt/ros/noetic/setup.bash; catkin_make

# Example building ROS 2 workspace in dockerfile
RUN cd ~/ros2_ws; source /opt/ros/foxy/setup.bash; colcon build

CMD ["bash"]

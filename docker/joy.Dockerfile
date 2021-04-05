FROM ros:foxy-ros-core
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-joy
CMD ros2 run joy joy_node
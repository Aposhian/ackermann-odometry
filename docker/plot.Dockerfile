FROM ros:foxy-ros-core
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-plotjuggler-ros
CMD ros2 run plotjuggler plotjuggler
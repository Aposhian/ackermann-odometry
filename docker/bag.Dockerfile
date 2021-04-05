FROM ros:foxy-ros-core
RUN apt-get update && apt-get install -y \
  ros-${ROS_DISTRO}-ros2bag \
  ros-${ROS_DISTRO}-rosbag2-converter-default-plugins \
  ros-${ROS_DISTRO}-rosbag2-storage-default-plugins
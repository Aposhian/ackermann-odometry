FROM ros:foxy-ros-core
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-plotjuggler-ros
ARG LAYOUT_FILE=plotjuggler-layout.xml
COPY ${LAYOUT_FILE} .
ENV LAYOUT_FILE=${LAYOUT_FILE}
CMD ros2 run plotjuggler plotjuggler --layout ${LAYOUT_FILE}
ARG WORKSPACE=/opt/ackermann_ws

FROM ros:foxy as builder
ARG WORKSPACE
WORKDIR ${WORKSPACE}
COPY ./src/ackermann_interfaces ./src/ackermann_interfaces
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

FROM ros:foxy-ros-core
ARG WORKSPACE
RUN apt-get update && apt-get install -y \
  ros-${ROS_DISTRO}-plotjuggler-ros \
  ros-${ROS_DISTRO}-ackermann-msgs

# project-specific interfaces
COPY --from=builder ${WORKSPACE}/install ${WORKSPACE}/install
ENV WORKSPACE=${WORKSPACE}
RUN sed -i --expression '$isource "$WORKSPACE/install/setup.bash"' /ros_entrypoint.sh

# plotjuggler layout setup
ARG LAYOUT_FILE=plotjuggler-layout.xml
ENV LAYOUT_FILE=${LAYOUT_FILE}
COPY ${LAYOUT_FILE} .

CMD ros2 run plotjuggler plotjuggler --layout ${LAYOUT_FILE}
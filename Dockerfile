ARG WORKSPACE=/opt/ackermann_ws

FROM ros:foxy
ARG WORKSPACE
WORKDIR ${WORKSPACE}
RUN apt-get update
COPY src src
RUN rosdep install -i --from-paths src --rosdistro foxy -y -q --ignore-src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build
ENV WORKSPACE ${WORKSPACE}
RUN sed -i --expression '$isource "$WORKSPACE/install/setup.bash"' /ros_entrypoint.sh

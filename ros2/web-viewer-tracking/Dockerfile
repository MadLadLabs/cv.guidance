FROM ros:foxy

ARG ROS_DISTRO=foxy
ENV ROS_DISTRO=$ROS_DISTRO
ENV ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3

COPY entrypoint.sh /
COPY src/app/web_viewer_tracking /webapp/web_viewer_tracking
COPY src/static /webapp-static
COPY src/templates /webapp-templates

WORKDIR /webapp

RUN colcon build


RUN sed --in-place --expression \
      '$isource "/webapp/install/setup.bash"' \
      /ros_entrypoint.sh

CMD [ "ros2", "run", "web_viewer_tracking", "web_viewer_node" ]
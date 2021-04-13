FROM ros:foxy

ARG ROS_DISTRO=foxy
ENV ROS_DISTRO=$ROS_DISTRO
ENV ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3

RUN apt-get update && apt-get install -y \
    python3-dev \
    python3-pip
    # ffmpeg libsm6 libxext6  # these will come in handy for the opencv ros2 package

RUN pip3 install flask

COPY src/app/web_viewer_tracking /webapp/web_viewer_tracking
COPY src/static /webapp-static
COPY src/templates /webapp-templates

WORKDIR /webapp

RUN colcon build

RUN sed --in-place --expression \
      '$isource "/webapp/install/setup.bash"' \
      /ros_entrypoint.sh

EXPOSE 8080

CMD [ "ros2", "run", "web_viewer_tracking", "web_viewer_node" ]
FROM ros:foxy

RUN apt-get update && apt-get install -y \
    python3-dev \
    python3-pip

RUN apt-get update && apt-get install -y \
    ffmpeg \
    libsm6 \
    libxext6

RUN apt-get update && apt-get install -y \
    ros-foxy-cv-bridge

RUN pip3 install \
    numpy \
    opencv-contrib-python==3.4.13.47 \
    imutils

COPY ../image_pub_sub_example /ros2_packages/image_pub_sub_example

WORKDIR /ros2_packages

RUN . /opt/ros/foxy/setup.sh && \
    colcon build

COPY ../images /images

RUN sed --in-place --expression \
      '$isource "/ros2_packages/install/setup.bash"' \
      /ros_entrypoint.sh

CMD [ "ros2", "run", "image_pub_sub_example", "publisher_node", "--ros-args", "-p", "image_path:=/images/dogs.jpeg" ]
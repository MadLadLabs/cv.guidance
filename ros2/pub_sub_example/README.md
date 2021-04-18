## Simple ROS2 docker pub-sub example

Build using

`docker build -t pub_sub_publisher -f Dockerfile.publisher .`

`docker build -t pub_sub_subscriber -f Dockerfile.subscriber .`

Then, run using

`docker run pub_sub_publisher`

`docker run pub_sub_subscriber`
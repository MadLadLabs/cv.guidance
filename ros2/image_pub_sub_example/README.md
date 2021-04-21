## How to run this example via docker-compose

assuming you have docker-compose + docker, you can `cd` into this directory and run

`docker-compose up`

This command will take some time to run as the container images have to be built first.

## How to run this example with docker only

Without docker-compose, build using

`docker build -t image_pub_sub_publisher -f Dockerfile.publisher .`

`docker build -t image_pub_sub_subscriber -f Dockerfile.subscriber .`

Then, run using

`docker run image_pub_sub_publisher`

`docker run -p 8080:8080 image_pub_sub_subscriber`

## For reference

Package created using 

`ros2 pkg create --build-type ament_python image_pub_sub_example`
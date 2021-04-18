Package created using 

`ros2 pkg create --build-type ament_python image_pub_sub_example`

Build using

`docker build -t image_pub_sub_publisher -f Dockerfile.publisher .`

`docker build -t image_pub_sub_subscriber -f Dockerfile.subscriber .`

Then, run using

`docker run image_pub_sub_publisher`

`docker run -p 8080:8080 image_pub_sub_subscriber`
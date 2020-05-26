# Dockerfiles for the exercise

Until OSRF adds official ROS Noetic images, we have to build them by ourself. 


Run from main directory

```
docker build --pull --rm -f "docker/Dockerfile-ros-noetic" -t ros:noetic "docker"

```

Note: Currently does not compile due to unavailability of Focal Jderobot Packages
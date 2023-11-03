# gets you a docker container where you can run ROS commands, so you don't have to install ROS on your machine
# mounts the current directory to /ros_workspaces/ballyer so you can access it in the container
docker run -v .:/ros_workspaces/ballyer -it ros:noetic
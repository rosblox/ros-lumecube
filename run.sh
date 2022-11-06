docker run -it --net=host --volume $(pwd)/ros_lumecube:/colcon_ws/src/ros_lumecube --volume /var/run/dbus/:/var/run/dbus/ ghcr.io/rosblox/ros-lumecube:humble

FROM ros:humble-ros-core

RUN apt-get update && apt-get install -y \
    python3-pip libglib2.0-dev \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install bluepy


WORKDIR /colcon_ws/src

COPY ros_lumecube ros_lumecube

WORKDIR /colcon_ws

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install

WORKDIR /

ENV FASTRTPS_DEFAULT_PROFILES_FILE=/fastrtps_profile.xml
COPY fastrtps_profile.xml .

COPY ros_entrypoint.sh .

RUN echo 'alias build="colcon build --cmake-args --symlink-install  --event-handlers console_direct+"' >> ~/.bashrc
RUN echo 'alias run="ros2 run ros_lumecube lumecube_node"' >> ~/.bashrc

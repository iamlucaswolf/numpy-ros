FROM osrf/ros:melodic-desktop-full

# Change shell to Bash
SHELL [ "/bin/bash" , "-c" ]

# TODO install from poetry

# Install Python 3 + basic utilities
RUN apt update \ 
    && apt install --no-install-recommends -y python3 \
        python3-pip \
        python3-setuptools \
        python3-wheel \
        less \
        tmux \
    && pip3 install numpy pyyaml rospkg catkin_pkg
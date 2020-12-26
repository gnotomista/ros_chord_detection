FROM ros:kinetic-ros-base-xenial

# install ros
RUN apt-get update && apt-get install -y \
    ros-kinetic-perception=1.3.2-0* \
    && rm -rf /var/lib/apt/lists/*

# create ros workspace
RUN mkdir -p /home/catkin_ws/src && cd /home/catkin_ws/src

# install dependencies for python node (mic capture)
RUN apt-get update && apt-get install -y portaudio19-dev python-pyaudio
RUN apt-get update && apt-get install -y python-matplotlib

# install dependencies for C++ node (chord detection)
RUN apt-get update && apt-get install -y fftw3-dev

# copy in init_build_and_start script
COPY ./linked_folder/init_build_and_start /home/init_build_and_start

# remove not working ros entrypoint
RUN rm /ros_entrypoint.sh

ENTRYPOINT ["/bin/bash"]

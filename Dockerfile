FROM nvidia/cuda:11.8.0-devel-ubuntu20.04

RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -sSL http://packages.ros.org/ros.key | apt-key add -
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

RUN apt-get update && apt-get install -y ros-noetic-ros-core \
  ros-noetic-roscpp \
  ros-noetic-sensor-msgs \
  ros-noetic-stereo-msgs \
  ros-noetic-image-transport \
  ros-noetic-cv-bridge \
  ros-noetic-image-view \
  libopencv-dev \
  build-essential \
  cmake \
  git \
  python3-pip \
  python3-catkin-tools && \
  rm -rf /var/lib/apt/lists/*

RUN pip3 install numpy opencv-python

ENV ROS_DISTRO=noetic
ENV ROS_PACKAGE_PATH=/opt/ros/noetic/share
ENV PATH=$PATH:/opt/ros/noetic/bin

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

WORKDIR /catkin_ws/src
COPY . /catkin_ws/src/semiglobal-matching/

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /catkin_ws && \
    catkin init && \
    catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build"

RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]

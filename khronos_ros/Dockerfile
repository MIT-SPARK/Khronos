# Use the official ROS Noetic base image
FROM osrf/ros:noetic-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary dependencies
RUN apt-get update && \
    apt-get install -y \
    git \
    xauth \
    x11-apps \
    nano \
    cmake \
    python3-catkin-tools \
    python3-vcstool \
    python3-tk \
    libepoxy-dev \
    ros-noetic-tf2-eigen \
    qtbase5-dev \
    libpcl-dev \
    libgoogle-glog-dev \
    ros-noetic-tf2-eigen \
    build-essential \
    nlohmann-json3-dev
#    rm -rf /var/lib/apt/lists/*

# Ensure .Xauthority exists and set correct permissions
RUN touch ~/.Xauthority && \
    ls -la ~/.Xauthority && \
    chmod 644 ~/.Xauthority

# Initialize rosdep if not already initialized
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rosdep init && rosdep fix-permissions &&  rosdep update; \
    else \
    rosdep fix-permissions && rosdep update; \
    fi

# Create and initialize the Khronos workspace
RUN mkdir -p ~/khronos_ws/src
WORKDIR /root/khronos_ws

# Initialize catkin workspace with configuration
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin init && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DOPENGV_BUILD_WITH_MARCH_NATIVE=OFF && \
    catkin config --extend /opt/ros/noetic"

RUN cd ~/khronos_ws/src && \
    git clone https://github.com/MIT-SPARK/Khronos.git khronos

WORKDIR /root/khronos_ws/src

#RUN sed -i '1i\
#- git:\n    local-name: gtsam\n    uri: https://github.com/borglab/gtsam.git\n    version: develop\n' /root/khronos_ws/src/khronos/install/https.rosinstall

# USE VCS TO INSTALL PACKAGES FROM HTTPS.ROSINSTALL

RUN vcs import . < ~/khronos_ws/src/khronos/install/https.rosinstall

RUN rm -rf /khronos_ws/src/pangolin

RUN sudo apt-get update

WORKDIR /root/khronos_ws
RUN rosdep update &&  rosdep fix-permissions && \
          rosdep install --from-paths src --ignore-src -r -y

# [Fix] Error: GTSAM was built against a different version of Eigen 
RUN rm -rf ~/khronos_ws/src/gtsam && apt update && apt install ros-noetic-gtsam

# [Fix] :
# Errors << kimera_vio_ros:make /catkin_ws/logs/kimera_vio_ros/build.make.000.log
# /usr/bin/ld: cannot find -lpango_core
# /usr/bin/ld: cannot find -lpango_display
# /usr/bin/ld: cannot find -lpango_geometry
# ......
WORKDIR /root/
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git

RUN cd /root/Pangolin
RUN yes | /root/Pangolin/scripts/install_prerequisites.sh required

RUN sudo apt-get update
RUN yes | sudo apt-get install python3-pip

WORKDIR /root/Pangolin
RUN mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install

RUN cd ~/khronos_ws && \
   catkin build khronos_ros

WORKDIR /root/khronos_ws/

CMD ["/bin/bash"]

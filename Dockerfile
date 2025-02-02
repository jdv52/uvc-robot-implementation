FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y\
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
    wget \
    git \
    cmake \
    stlink-tools \
    libhidapi-hidraw0 \
    libusb-1.0-0 \
    libhidapi-dev \
    libusb-1.0-0-dev \
    libusb-dev \
    libtool \ 
    usbutils \
    make \
    automake \
    pkg-config \
    tclsh \
    telnet \
    && apt-get install -y lsb-release gnupg 

RUN apt-get update \
    && apt-get install -y ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-rqt-image-view \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gz-ros2-control

RUN apt-get update \
    && apt-get install -y sudo \
    && echo ubuntu ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/ubuntu \
    && chmod 0440 /etc/sudoers.d/ubuntu \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp

# Install compiler
RUN wget https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v12.2.1-1.2/xpack-arm-none-eabi-gcc-12.2.1-1.2-linux-x64.tar.gz
RUN tar xf xpack-arm-none-eabi-gcc-12.2.1-1.2-linux-x64.tar.gz
RUN mkdir gcc-arm-none-eabi && cp -r xpack-arm-none-eabi-gcc-12.2.1-1.2/* gcc-arm-none-eabi/
RUN rm -rf xpack-arm-none-eabi-gcc-12.2.1-1.2
RUN rm xpack-arm-none-eabi-gcc-12.2.1-1.2-linux-x64.tar.gz
ENV PATH="${PATH}:/home/dev/gcc-arm-none-eabi/bin"

# # Build openocd from source
# RUN git clone https://github.com/openocd-org/openocd.git \
#   && cd openocd \
#   && ./bootstrap \ 
#   && ./configure --enable-stlink \
#   && make -j"$(nproc)" \
#   && make install-strip \
#   && cd .. \
#   && rm -rf openocd

COPY entrypoint.sh /entrypoint.sh

WORKDIR /home/ubuntu/ws/

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ubuntu/.bashrc
RUN echo "source /usr/share/gazebo/setup.bash" >> /home/ubuntu/.bashrc

CMD ["bash"]
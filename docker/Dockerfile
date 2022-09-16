#FROM ubuntu:jammy
FROM arm64v8/ubuntu:jammy 
ARG DEBIAN_FRONTEND=noninteractive

# copy all the files to the container
ADD ./ros2_interface/ /root/ros2_ws/src

# setup entrypoint
COPY ./ros_entrypoint.sh /
RUN bash -c "chmod +x /ros_entrypoint.sh"

# Set locale
RUN apt-get update && apt-get install -y locales && \
    apt-get install apt-utils && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Setup source
RUN apt-get install -y software-properties-common && \
	add-apt-repository -y universe && \
    apt-get update && apt-get install -y curl gnupg lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" |  tee /etc/apt/sources.list.d/ros2.list > /dev/null 

# Install ROS2 packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y ros-humble-desktop && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# create environment folder
RUN mkdir -p /root/ros2_ws/src

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
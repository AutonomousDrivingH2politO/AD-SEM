FROM ubuntu:22.04

RUN apt-get update -y && \
    apt-get install locales -y && \
    locale-gen en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV TZ=Europe/Rome
ENV DEBIAN_FRONTEND=noninteractive 

RUN apt-get install software-properties-common -y && \
    add-apt-repository universe

RUN apt-get update -y && \
    apt-get install curl -y && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && \
    echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb

RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install ros-humble-desktop -y

RUN useradd -m -s /bin/bash carlauser

RUN echo "source /opt/ros/humble/setup.bash" >> /home/carlauser/.bashrc 

COPY --from=carlasim/carla:0.9.16 /workspace /home/carlauser/carla
WORKDIR /home/carlauser/carla

RUN apt-get install python3-pip python3-venv git -y && \
    python3 -m venv . && \
    . ./bin/activate && \
    pip install -r PythonAPI/examples/requirements.txt

USER carlauser

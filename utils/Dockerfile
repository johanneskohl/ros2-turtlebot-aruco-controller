ARG _DISTRO=jazzy
FROM ros:${_DISTRO}-ros-base
SHELL ["/bin/bash", "-c"]

RUN set -x && \
  apt-get update -y -qq && \
  apt-get upgrade -y -qq && \
  apt-get install -y -qq\
    sudo \
    dkms \
    gdb \
    wget curl zip unzip tar \
    nano \
    acl \
    python3-venv python3-pip \
    v4l-utils \
    git openssh-client && \
  apt-get autoremove -y -qq

ARG _DISTRO
ARG _USERNAME=ubuntu
ARG _GROUPNAME=${_USERNAME}
ARG _USERID=1000
ARG _GROUPID=${_USERID}

### Create the user
RUN set -x && \
  if ! gId=$(getent group ${_GROUPNAME} 2> /dev/null | cut -d: -f3); then \
    groupadd --gid ${_GROUPID} ${_GROUPNAME}; \
  else \
    test ${gId} -eq ${_GROUPID}; \
  fi && \
  if ! uId=$(id -u ${_USERNAME} 2> /dev/null); then \
    useradd --uid ${_USERID} --gid ${_GROUPID} -m ${_USERNAME}; \
  else \
    test ${uId} -eq ${_USERID}; \
  fi

# make user able to sudo w/o password
RUN set -x && \
  echo ${_USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${_USERNAME} && \
  chmod 0440 /etc/sudoers.d/${_USERNAME}

# install opencv
RUN set -x && \
  apt-get update -y -qq && \
  apt-get install -y -qq libopencv-dev

# install ros2 stuff
RUN set -x && \
  apt-get update -y -qq && \
  apt-get install -y -qq \
    ros-${_DISTRO}-gazebo-* \
    ros-${_DISTRO}-cartographer ros-${_DISTRO}-cartographer-ros \
    ros-${_DISTRO}-navigation2 ros-${_DISTRO}-nav2-bringup \
    ros-${_DISTRO}-tf-transformations \
    python3-colcon-common-extensions \
    apt-transport-https
RUN set -x && \
  apt-get update -y -qq && \
  apt-get install -y -qq ros-${_DISTRO}-desktop

WORKDIR /home/${_USERNAME}
USER ${_USERNAME}

RUN set -x && \
  sudo mkdir -p /opt/turtlebot3_ws/src && \
  sudo chown -R ${_USERID}:${_USERID} /opt/turtlebot3_ws/ && \
  cd /opt/turtlebot3_ws/src/ && \
  git clone -b ${_DISTRO} https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
  git clone -b ${_DISTRO} https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
  git clone -b ${_DISTRO} https://github.com/ROBOTIS-GIT/turtlebot3.git && \
  cd /opt/turtlebot3_ws && \
  source /opt/ros/${_DISTRO}/setup.bash && \
  colcon build --symlink-install

ARG _ROS_DOMAIN_ID
ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=${_ROS_DOMAIN_ID}

RUN set -x && \
  echo "alias x='exit'" >> ~/.bash_aliases && \
  echo "alias c='clear'" >> ~/.bash_aliases && \
  touch ~/.sudo_as_admin_successful && \
  echo "source /opt/ros/${_DISTRO}/setup.bash" >> ~/.bashrc && \
  echo "source /opt/turtlebot3_ws/install/local_setup.bash" >> ~/.bashrc && \
  echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
RUN set -x && \
  sudo usermod -a -G video ${_USERNAME}

CMD ["/bin/bash"]
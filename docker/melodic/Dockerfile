FROM ros:melodic
ARG USER
ENV USER ${USER}
ENV HOME /home/${USER}
ENV ROS_WS ${HOME}/catkin_ws
ENV PASS password
# ENV ROBOT_MODEL_NAME VS060A3-AV6-W4N-ANN

ENV DEBIAN_FRONTEND=noninteractive

# RUN apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
# RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

RUN apt-get update
RUN apt-get install -y -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" \
    vim tmux git wget iputils-ping net-tools \
    ros-melodic-ros-tutorials \
    ros-melodic-rqt ros-melodic-rqt-common-plugins \
    ros-melodic-rviz \
    ros-melodic-rviz-visual-tools \
    ros-melodic-ros-control ros-melodic-ros-controllers \ 
    ros-melodic-moveit \
    ros-melodic-moveit-visual-tools \
    ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control \
    ros-melodic-robot-state-publisher \
    ros-melodic-joint-state-publisher \
    ros-melodic-joint-state-publisher-gui \
    ros-melodic-image-* \
    ros-melodic-rqt-image-view \
    # ros-melodic-usb-cam \
    ros-melodic-uvc-camera \
    # librealsense2-dkms \
    # librealsense2-utils \
    # librealsense2-dev \
    # librealsense2-dbg \
    ros-melodic-librealsense2 \
    ros-melodic-librealsense2-dbgsym \
    ros-melodic-realsense2-camera \
    ros-melodic-realsense2-camera-dbgsym \
    # use extended repogitory for gazebo instead of following
    ros-melodic-realsense2-description \
    python-catkin-tools

RUN mkdir -p ${ROS_WS}/src
# RUN git clone -b melodic-devel https://github.com/DENSORobot/denso_robot_ros.git ~/catkin_ws/src
# RUN git clone https://github.com/ros/urdf_tutorial.git ~/catkin_ws/src/urdf_tutorial
# RUN git clone https://github.com/osrf/gazebo_models ~/.gazebo/models
# RUN sed -i s/VS060A3-AV6-NNN-NNN/${ROBOT_MODEL_NAME}/ ~/catkin_ws/src/denso_robot_descriptions/vs060_description/vs060.launch.xml
# gazebo起動時のRESTエラー回避
# RUN sed -i s/fuel/robotics/ ~/.ignition/fuel/config.yaml

RUN touch ~/.bashrc
RUN echo "source /ros_entrypoint.sh" >> ~/.bashrc
RUN echo "set +e" >> ~/.bashrc

RUN cd ${ROS_WS} && /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin build"
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:~/catkin_ws" >> ~/.bashrc
# RUN echo "export ROS_WORKSPACE=~/catkin_ws" >> ~/.bashrc

RUN useradd -s /bin/bash ${USER}
RUN usermod -d ${HOME} ${USER}
RUN gpasswd -a ${USER} sudo
RUN chown -R ${USER} ${HOME}
RUN echo "${USER}:${PASS}" | chpasswd
# /dev/video0のPermission Denied回避
RUN usermod -aG sudo,video ${USER}

RUN mkdir -p ${HOME}/.gazebo/models
COPY ./models/ ${HOME}/.gazebo/models/
RUN chmod 777 ${HOME}/.gazebo

# gazebo起動後に生成される設定ファイルのURLエラー回避
RUN  mkdir -p ~/.ignition/fuel
COPY ./configs/ignition_robotics/config.yaml ${HOME}/.ignition/fuel/

USER ${USER}
WORKDIR ${ROS_WS}

# エイリアス
RUN echo 'alias ccp="catkin_create_pkg"' >> ~/.bashrc
RUN echo 'alias cb="catkin build"' >> ~/.bashrc
RUN echo 'alias rl="roslaunch"' >> ~/.bashrc

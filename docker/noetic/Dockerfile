FROM ros:noetic
ARG USER
ENV USER ${USER}
ENV HOME /home/${USER}
ENV ROS_WS ${HOME}/catkin_ws
ENV PASS password

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update
# キーボードレイアウト選択回避 | ref: https://qiita.com/DS27/items/7301cce7a2c1dec107ad
RUN apt-get install -y -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" \
    vim tmux git wget iputils-ping net-tools python3-catkin-tools \
    ros-noetic-rqt ros-noetic-rqt-common-plugins \
    ros-noetic-rviz \
    ros-noetic-rviz-visual-tools \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-image-* \
    ros-noetic-rqt-image-view 

RUN mkdir -p ${ROS_WS}/src

RUN touch ~/.bashrc
RUN echo "source /ros_entrypoint.sh" >> ~/.bashrc
RUN echo "set +e" >> ~/.bashrc

RUN cd ${ROS_WS} && /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin build"
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:~/catkin_ws" >> ~/.bashrc

RUN useradd -s /bin/bash ${USER}
RUN usermod -d ${HOME} ${USER}
RUN gpasswd -a ${USER} sudo
RUN chown -R ${USER} ${HOME}
RUN echo "${USER}:${PASS}" | chpasswd
# /dev/video0のPermission Denied回避
RUN usermod -aG sudo,video ${USER}

USER ${USER}
WORKDIR ${ROS_WS}

# エイリアス
RUN echo 'alias ccp="catkin_create_pkg"' >> ~/.bashrc
RUN echo 'alias cb="catkin build"' >> ~/.bashrc
RUN echo 'alias rl="roslaunch"' >> ~/.bashrc
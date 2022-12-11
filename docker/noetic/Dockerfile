FROM sin392/ros_noetic_base:latest

ARG BUILD_USER
ENV USER ${BUILD_USER}
ENV HOME /home/${USER}
ENV ROS_WORKSPACE ${HOME}/catkin_ws
ENV PATH=${HOME}/.local/bin:${PATH}

USER root
RUN usermod -l ${USER} appuser
RUN usermod -d /home/${USER} -m ${USER}
RUN usermod -c ${USER} ${USER}
RUN sed -i s/appuser/${USER}/ ${HOME}/.bashrc
RUN rm -R -f /home/appuser

USER ${USER}
WORKDIR ${ROS_WORKSPACE}
# WARN: ユーザを切り替えるとdetectron2消えちゃってるかもしれないので再インストール
# RUN pip install --user -e detectron2_repo
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin build"

# エイリアス
RUN echo 'alias ccp="catkin_create_pkg"' >> ${HOME}/.bashrc
RUN echo 'alias cb="catkin build"' >> ${HOME}/.bashrc
RUN echo 'alias rl="roslaunch"' >> ${HOME}/.bashrc
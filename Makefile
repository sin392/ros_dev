ip_address= 
force_build=false

init:
	# melodic
	# git clone -b melodic-devel https://github.com/DENSORobot/denso_robot_ros.git ./src/melodic/public/denso_robot_ros
	git clone -b melodic-devel-vs087 https://github.com/sin392/denso_robot_ros.git ./src/melodic/public/denso_robot_ros
	# git clone https://github.com/IntelRealSense/realsense-ros.git ./src/melodic/public/realsense_ros
	git clone https://github.com/ros/urdf_tutorial.git ./src/melodic/public/urdf_tutorial
	git clone https://github.com/issaiass/realsense2_description.git ./src/melodic/public/realsense2_description
	git clone https://github.com/issaiass/realsense_gazebo_plugin.git ./src/melodic/public/realsense_gazebo_plugin
	# git clone -b melodic-devel https://github.com/introlab/find-object.git ./src/melodic/public/find-object
	sed -i s/VS060A3-AV6-NNN-NNN/${ROBOT_MODEL_NAME}/ ./src/melodic/public/denso_robot_ros/denso_robot_descriptions/vs060_description/vs060.launch.xml
	# noetic
	git clone https://github.com/sin392/detect.git ./src/noetic/detect

	docker network create ros_dev_external

ifeq ($(force_build), true)
	docker build -f docker/melodic/Dockerfile.base -t sin392/ros_melodic_base:latest .
	docker build -f docker/noetic/Dockerfile.base -t sin392/ros_noetic_base:latest .
else
	docker pull sin392/ros_melodic_base:latest
	docker pull sin392/ros_noetic_base:latest
endif

start:
	xhost + localhost
ifdef $(ip_address)
	ROS_MASTER_DOMAIN=$(ip_address) docker-compose up -d --force-recreate
	roscore
else
	ROS_MASTER_DOMAIN=ros_melodic docker-compose up -d --force-recreate
endif

start-host:
	xhost + localhost
	# localhostなどでは失敗する | ref: https://www.finnrietz.dev/linux/ros-docker/
	ROS_MASTER_DOMAIN=${ROS_IP} docker-compose up -d --force-recreate
	roscore

stop:
	xhost - localhost
	docker-compose stop

melodic:
	docker-compose exec ros_melodic /bin/bash

noetic:
	docker-compose exec ros_noetic /bin/bash
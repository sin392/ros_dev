init:
	# git clone -b melodic-devel https://github.com/DENSORobot/denso_robot_ros.git ./src/melodic/public/denso_robot_ros
	git clone -b melodic-devel-vs087 https://github.com/sin392/denso_robot_ros.git ./src/melodic/public/denso_robot_ros
	# git clone https://github.com/IntelRealSense/realsense-ros.git ./src/melodic/public/realsense_ros
	git clone https://github.com/ros/urdf_tutorial.git ./src/melodic/public/urdf_tutorial
	git clone https://github.com/issaiass/realsense2_description.git ./src/melodic/public/realsense2_description
	git clone https://github.com/issaiass/realsense_gazebo_plugin.git ./src/melodic/public/realsense_gazebo_plugin
	git clone -b melodic-devel https://github.com/introlab/find-object.git ./src/melodic/public/find-object
	sed -i s/VS060A3-AV6-NNN-NNN/${ROBOT_MODEL_NAME}/ ./src/melodic/public/denso_robot_ros/denso_robot_descriptions/vs060_description/vs060.launch.xml

	docker network create ros_dev_external
	docker-compose build

start:
	xhost + localhost
	docker-compose up -d

stop:
	xhost - localhost
	docker-compose stop

melodic:
	docker-compose exec ros_melodic /bin/bash

noetic:
	docker-compose exec ros_noetic /bin/bash
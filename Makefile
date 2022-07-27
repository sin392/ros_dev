init:
	# git clone -b melodic-devel https://github.com/DENSORobot/denso_robot_ros.git ./src/public/denso_robot_ros
	git clone -b melodic-devel-vs087 https://github.com/sin392/denso_robot_ros.git ./src/public/denso_robot_ros
	# git clone https://github.com/IntelRealSense/realsense-ros.git ./src/public/realsense_ros
	git clone https://github.com/ros/urdf_tutorial.git ./src/public/urdf_tutorial
	git clone https://github.com/issaiass/realsense2_description.git ./src/public/realsense2_description
	git clone https://github.com/issaiass/realsense_gazebo_plugin.git ./src/public/realsense_gazebo_plugin
	sed -i s/VS060A3-AV6-NNN-NNN/${ROBOT_MODEL_NAME}/ ./src/public/denso_robot_ros/denso_robot_descriptions/vs060_description/vs060.launch.xml

	docker network create ros_dev_external
	docker-compose build

start:
	xhost + localhost
	docker-compose up -d

stop:
	xhost - localhost
	docker-compose stop

shell:
	docker-compose exec ros_dev /bin/bash
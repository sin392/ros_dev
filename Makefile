init:
	docker network create ros_dev_external
	docker-compose build

start:
	open -a xquartz
	xhost + localhost
	docker-compose up -d

stop:
	killall "Xquartz"
	xhost - localhost
	docker-compose stop

shell:
	docker-compose exec ros_dev /bin/bash
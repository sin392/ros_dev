init:
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
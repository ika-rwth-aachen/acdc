all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build     - build image"
	@echo "   2. make push      - push image"
	@echo "   3. make pull 	    - pull image"
	@echo "   4. make clean     - remove image"
	@echo ""

build_ros1:
	@docker buildx build \
		--push \
		--file Dockerfile.ros1 \
		--platform linux/amd64,linux/arm64 \
		--cache-from type=registry,ref=rwthika/acdc:ros1-cuda-tf \
		--tag rwthika/acdc:ros1-cuda-tf \
		.


build_ros2:
	@docker buildx build \
		--push \
		--file Dockerfile.ros2 \
		--cache-from type=registry,ref=rwthika/acdc:ros2-cuda-tf \
		--tag rwthika/acdc:ros2-cuda-tf \
		.

pull_ros1:
	@docker pull rwthika/acdc:ros1-cuda-tf

clean_ros1:
	@docker rmi -f rwthika/acdc:ros1-cuda-tf

pull_ros2:
	@docker pull rwthika/acdc:ros2-cuda-tf

clean_ros2:
	@docker rmi -f rwthika/acdc:ros2-cuda-tf

all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build     - build image"
	@echo "   2. make push      - push image"
	@echo "   3. make pull 		- pull image"
	@echo "   4. make clean     - remove image"
	@echo ""

	
build: # build for current platform 
	@docker buildx build \
		--load \
		--file Dockerfile.humble \
		--cache-from type=registry,ref=rwthika/acdc:ros2-cuda-tf \
		--tag rwthika/acdc:ros2-cuda-tf \
		.

push: # build and push for both platforms
	@docker buildx build \
		--push \
		--file Dockerfile.humble \
		--platform linux/amd64,linux/arm64 \
		--cache-from type=registry,ref=rwthika/acdc:ros2-cuda-tf \
		--tag rwthika/acdc:ros2-cuda-tf \
		.

pull:
	@docker pull rwthika/acdc:ros2-cuda-tf

clean:
	@docker rmi -f rwthika/acdc:ros2-cuda-tf

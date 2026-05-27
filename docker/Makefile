IMAGE := vins-fusion-ros2

all: help

help:
	@echo ""
	@echo "-- Help Menu (run from project root)"
	@echo ""
	@echo "   make build   - build the Docker image"
	@echo "   make clean   - remove the Docker image"
	@echo ""

build:
	docker build --tag $(IMAGE) -f docker/Dockerfile .

clean:
	docker rmi -f $(IMAGE)

.PHONY: all help build clean

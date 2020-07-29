.PHONY: docker_test
docker_test:
	docker build --build-arg uid=$(shell id -u) -t docker_test .

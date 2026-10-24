.PHONY: image up down shell build mock headless smoke smoke-mock logs

image:
	docker compose build
up:
	docker compose up -d
down:
	docker compose down
shell:
	docker compose exec --user ros dev /usr/local/bin/ros-shell.sh bash
build:
	docker compose run --rm --no-deps dev bash scripts/build.sh
mock:
	SIM_MODE=mock GUI=false docker compose up -d
headless:
	GUI=false RVIZ=false ENABLE_DESKTOP=0 docker compose up -d
smoke:
	docker compose run --rm --no-deps -e ROS_DOMAIN_ID=43 dev bash scripts/smoke-test.sh gazebo
smoke-mock:
	docker compose run --rm --no-deps -e ROS_DOMAIN_ID=44 dev bash scripts/smoke-test.sh mock
logs:
	docker compose logs -f dev

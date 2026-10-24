.PHONY: image up down shell build mock headless demo obstacles perception smoke smoke-mock smoke-obstacles bridge-check registration-check rates-check pick-check camera-check imu-check lidar-motion-check logs

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
demo:
	ROBOT_MODEL=demo docker compose up -d
obstacles:
	TEST_WORLD=obstacle_course docker compose up -d
perception:
	ENABLE_PERCEPTION=true docker compose up -d
smoke:
	docker compose run --rm --no-deps -e ROS_DOMAIN_ID=43 dev bash scripts/smoke-test.sh gazebo
smoke-mock:
	docker compose run --rm --no-deps -e ROS_DOMAIN_ID=44 dev bash scripts/smoke-test.sh mock
smoke-obstacles:
	TEST_WORLD=obstacle_course docker compose run --rm --no-deps -e ROS_DOMAIN_ID=45 dev bash scripts/smoke-test.sh gazebo
bridge-check:
	docker compose exec --user ros dev /usr/local/bin/ros-shell.sh python3 scripts/check_bridge.py
registration-check:
	docker compose exec --user ros dev /usr/local/bin/ros-shell.sh bash -c 'python3 scripts/check_registration.py --world "$${TEST_WORLD:-workcell}"'
rates-check:
	docker compose exec --user ros dev /usr/local/bin/ros-shell.sh python3 scripts/check_rates.py --min-hz "$${LIDAR_HZ:-10.0}"
pick-check:
	docker compose exec --user ros dev /usr/local/bin/ros-shell.sh bash -c 'python3 scripts/check_pick.py --part pick_cube && python3 scripts/check_pick.py --part pick_cylinder'
camera-check:
	docker compose exec --user ros dev /usr/local/bin/ros-shell.sh bash -c 'python3 scripts/check_realsense.py --camera wrist && python3 scripts/check_realsense.py --camera overhead'
imu-check:
	docker compose exec --user ros dev /usr/local/bin/ros-shell.sh python3 scripts/check_imu.py
lidar-motion-check:
	docker compose exec --user ros dev /usr/local/bin/ros-shell.sh python3 scripts/check_lidar_motion.py
logs:
	docker compose logs -f dev

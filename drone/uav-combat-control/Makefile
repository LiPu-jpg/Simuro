PYTHON ?= python3
LAUNCH_ARGS ?= headless:=true run_rosbag:=false

.PHONY: check pack build run clean

check:
	$(PYTHON) -I tools/pack_submission.py --source "$(CURDIR)" --check-only

pack:
	$(PYTHON) -I tools/pack_submission.py --source "$(CURDIR)"

build:
	bash -c 'source /opt/ros/humble/setup.bash && colcon build --base-paths . --packages-select uav_combat_control --symlink-install'

run: build
	bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch uav_combat_control run_match.launch.py $(LAUNCH_ARGS)'

clean:
	rm -rf build install log dist *.egg-info

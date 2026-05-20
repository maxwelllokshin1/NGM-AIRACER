colcon build --packages-select follow_the_gap

source install/setup.bash

ros2 launch follow_the_gap follow_the_gap.launch.py
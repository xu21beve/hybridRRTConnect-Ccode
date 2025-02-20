# !/bin/bash
./../../build/examples/multicopter3
echo "0 1" > "coordinate_indices.txt"
echo "y" > "obstacles.txt"
colcon build --packages-select visualize
source install/local_setup.bash
source /opt/ros/humble/setup.bash
ros2 run visualize ros_visualize
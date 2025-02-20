# !/bin/bash
./../../build/examples/bouncing_ball2
echo "4 0" > "coordinate_indices.txt"
echo "n" > "obstacles.txt"
colcon build --packages-select visualize
source install/local_setup.bash
source /opt/ros/humble/setup.bash
ros2 run visualize ros_visualize
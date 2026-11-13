source ~/unitree_ros2/setup.sh
source install/setup.bash 
ros2 run unitree_bridge unitree_bridge
ros2 topic pub /startNavi std_msgs/msg/Bool "{data: true}"
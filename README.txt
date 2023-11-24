export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_maze.launch.py 
ros2 launch nav2_bringup navigation_launch.py
ros2 launch slam_toolbox online_async_launch.py
rviz2


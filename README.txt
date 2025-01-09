export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_maze.launch.py 
ros2 launch nav2_bringup navigation_launch.py
ros2 launch slam_toolbox online_async_launch.py
rviz2


chmod +x nav2_final.py
./nav2_final.py

^^^^^^^^^^^^^^^
Commands needed to run the script
make sure the terminal is in the same location as the python script


bored in intern
i was here

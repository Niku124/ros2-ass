#!/usr/bin/env python3
import rclpy
import numpy as np
import time
import tf_transformations
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler



class MazeSolverNode(Node):
    def __init__(self):
        super().__init__('maze_solver_node')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.publisher_Twist = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_Pose = self.create_publisher(PoseStamped, 'move_base_simple/goal', 10)
        self.nav = BasicNavigator()
        self.sections = None


    def laser_callback(self, msg):
        # Get the laser scan data
        ranges = msg.ranges
        # Calculate the number of ranges
        num_ranges = len(ranges)

        # Split the ranges into twelve sections
        section = num_ranges // 12
        sections = [ranges[i * section:(i + 1) * section] for i in range(12)]

        # For example, print the average distance in each section
        for i in range(12):
            print(f"Section {12-i}: ", np.mean(sections[i]))
        # twist_msg = self.calculate_movement(sections)
        # self.publisher.publish(twist_msg)

        self.sections = sections
        # self.calculate_movement(sections,self.nav)



    def calculate_movement(self, sections,navigator: BasicNavigator):

        # Replace 'inf' values with a large number
        sections = [[distance if distance != float('inf') else 1e10 for distance in section] for section in sections]

        # Find the index of the section with the maximum average distance
        max_index = np.argmax([np.mean(section) for section in sections])

        # Calculate the angle
        angle = max_index * 2 * np.pi / 12

        # Create a PoseStamped message
        pose = PoseStamped()
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        q = quaternion_from_euler(0, 0, angle)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        # Publish the PoseStamped message
        self.publisher_Pose.publish(pose)

        # Use the navigator to go to the pose
        navigator.goToPose(pose)
        
        if navigator.isTaskComplete():
            print("Task complete")
            self.calculate_movement_flag = False                    
        
            

            

def main(args=None):
    rclpy.init(args=args)
    maze_solver_node = MazeSolverNode()
    #rclpy.spin(maze_solver_node)

    while rclpy.ok():

        # Calculate and execute the movement
        maze_solver_node.calculate_movement(maze_solver_node.sections, maze_solver_node.nav)
        # Wait for a short period of time before the next iteration
        rclpy.spin_once(maze_solver_node)



    
    maze_solver_node.destroy_node()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()

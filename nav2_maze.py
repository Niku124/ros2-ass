#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class MazeSolverNode(Node):
    def __init__(self):
        super().__init__('maze_solver_node')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

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
            print(f"Section {i + 1}: ", np.mean(sections[i]))

        twist_msg = self.calculate_movement(msg)
        self.publisher.publish(twist_msg)

    def calculate_movement(self, laser_data):
        # Implement algorithm logic based on laser scan data
        # Adjust robot's movements (e.g., turn left, turn right, move forward)

        
        twist_msg = Twist()
        # Example: Always move forward
        twist_msg.linear.x = 0.5
        return twist_msg
    

def main(args=None):
    rclpy.init(args=args)
    maze_solver_node = MazeSolverNode()
    rclpy.spin(maze_solver_node)
    maze_solver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

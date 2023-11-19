#!/usr/bin/env python3
import rclpy
import numpy as np
import math
import time
import tf_transformations
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus






class MazeSolverNode(Node):
    def __init__(self):
        super().__init__('maze_solver_node')
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.publisher_Twist = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_Pose = self.create_publisher(PoseStamped, 'move_base_simple/goal', 10)
        

        self.nav = BasicNavigator()
        self.current_pose = None
        self.avg_front = 0
        self.avg_back = 0
        self.avg_left = 0
        self.avg_right = 0
        self.current_x = 0
        self.current_y = 0
        self.current_z = 0
        self.current_orient_x = 0
        self.current_orient_y = 0
        self.current_orient_z = 0
        self.current_orient_w = 0




    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        self.current_orient_x = msg.pose.pose.orientation.x
        self.current_orient_y = msg.pose.pose.orientation.y
        self.current_orient_z = msg.pose.pose.orientation.z
        self.current_orient_w = msg.pose.pose.orientation.w


    def laser_callback(self, msg):
        ranges = msg.ranges
        num_ranges = len(ranges)
        section = num_ranges // 12
        sections = [ranges[i * section:(i + 1) * section] for i in range(12)]
        # for i in range(12):
        #     print(f"Section {12-i}: ", np.mean(sections[i]))

        self.avg_right = np.mean(sections[9])
        self.avg_left = np.mean(sections[3])
        self.avg_front = np.mean(sections[0])
        self.avg_back = np.mean(sections[6])
        


    def rotate(self, angle):
        rotation_matrix = tf_transformations.quaternion_matrix([self.current_orient_x, self.current_orient_y, self.current_orient_z, self.current_orient_w])
        turn_angle = math.radians(angle)
        turn_matrix = tf_transformations.rotation_matrix(turn_angle, [0, 0, 1])
        new_rotation_matrix = np.dot(turn_matrix, rotation_matrix)
        new_quaternion = tf_transformations.quaternion_from_matrix(new_rotation_matrix)
        return new_quaternion

    def publish_twist(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher_Twist.publish(twist)


    def navigate(self):

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.nav.get_clock().now().to_msg()

        print("Front:",self.avg_front, "Back:",self.avg_back, "Left:",self.avg_left, "Right:",self.avg_right)
        #Checks if there's a wall to the left
        if self.avg_left < 0.6 and self.nav.isTaskComplete():

            #Checks if infront has anything
            if self.avg_front > 0.5:
                self.publish_twist(0.1,0.0)
                print("Moving forward")

            elif self.avg_front < 0.5 and self.nav.isTaskComplete():
                new_quaternion = self.rotate(-100)
                goal.pose.orientation.z = new_quaternion[2]
                goal.pose.orientation.w = new_quaternion[3]
                goal.pose.position.x = self.current_x
                goal.pose.position.y = self.current_y
                goal.pose.position.z = self.current_z
                self.nav.goToPose(goal)     
                print("Turning Right")


        elif self.avg_left > 0.6 and self.nav.isTaskComplete():
                
                new_quaternion = self.rotate(95)
                goal.pose.orientation.z = new_quaternion[2]
                goal.pose.orientation.w = new_quaternion[3]
                goal.pose.position.x = self.current_x
                goal.pose.position.y = self.current_y
                goal.pose.position.z = self.current_z
                self.nav.goToPose(goal)     
                print("Turning left")

                if self.nav.isTaskComplete():
                    self.publish_twist(0.1,0.0)
                    print("Moving forward after turning left")


        
        if self.avg_front <0.5:
            self.publish_twist(0.0,0.0)
            print("Stopped")
            




            

def main(args=None):
    rclpy.init(args=args)
    maze_solver_node = MazeSolverNode()
    maze_solver_node.create_timer(1.0,maze_solver_node.navigate)
    rclpy.spin(maze_solver_node)
    
    
    maze_solver_node.destroy_node()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
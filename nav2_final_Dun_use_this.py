#!/usr/bin/env python3
import rclpy
import numpy as np
import math
import time
import tf_transformations
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped,Quaternion
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf_transformations import euler_from_quaternion, quaternion_from_euler






class MazeSolverNode(Node):
    def __init__(self):
        super().__init__('maze_solver_node')
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.publisher_Twist = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_Pose = self.create_publisher(PoseStamped, 'move_base_simple/goal', 10)
        self._action_client = ActionClient(self, NavigateToPose, 'NavigateToPose')


        self.nav = BasicNavigator()
        self.current_pose = None
        self.avg_front = 0
        self.avg_back = 0
        self.avg_left = 0
        self.avg_right = 0

        self.front = 0
        self.left = 0
        self.right = 0



        self.current_x = 0
        self.current_y = 0
        self.current_z = 0
        self.current_orient_x = 0
        self.current_orient_y = 0
        self.current_orient_z = 0
        self.current_orient_w = 0
        self.left_turn = 0
        self.right_turn = 0
        self.move_forward = 0

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
        self.avg_right = np.mean(sections[9])
        self.avg_left = np.mean(sections[3])
        self.avg_front = np.mean(sections[0])
        self.avg_back = np.mean(sections[6])

        self.current_2oclock = np.mean(sections[11])
        self.current_10oclock = np.mean(sections[2])

        self.right = np.mean(sections[8] + sections[9] + sections[10])
        self.left = np.mean(sections[2] + sections[3] + sections[4])
        self.front = np.mean(sections[0] + sections[1] + sections[11])

        

    # Rotate function using PoseStamped
    # It works but i'm getting this feeling that calling other functions might mess with the sensors callback
    # Idk but its traumatising
    def rotate(self, angle):
        angle_rad = math.radians(angle)
        quaternion = tf_transformations.quaternion_from_euler(0, 0, angle_rad)
        return quaternion



    def navigate(self):

        twist = Twist()
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.nav.get_clock().now().to_msg()
        print("Left Turn Status : ", self.left_turn)
        print("Right Turn Status : ", self.right_turn)
        print("Move Forward Status : ", self.move_forward)


        #Check for Left Wall
        if self.left < 0.9 :

            #Left Wall Detected
            #Check for Front Wall
            if self.front > 0.6:   
                twist.linear.x = 0.3
                self.publisher_Twist.publish(twist)                
                self.right_turn = 0
                print("LEFT DETECTED - MOVING FORWARD")

            #Front Wall Detected
            #right_turn flag is 0
            #Turn right
            elif self.front <0.6 and self.right_turn == 0:
                # Get the robot's current orientation in radians
                current_orient_rad = math.atan2(2.0 * (self.current_orient_w * self.current_orient_z + self.current_orient_x * self.current_orient_y), 
                                                1.0 - 2.0 * (self.current_orient_y * self.current_orient_y + self.current_orient_z * self.current_orient_z))
                # Calculate the new orientation for the right turn
                new_orient_rad = current_orient_rad - math.radians(103)
                # Calculate the quaternion from the new orientation
                quaternion = tf_transformations.quaternion_from_euler(0, 0, new_orient_rad)

                # Set goal.pose.orientation.z and goal.pose.orientation.w with the new quaternion
                goal.pose.orientation.z = quaternion[2]
                goal.pose.orientation.w = quaternion[3]
                goal.pose.position.x = self.current_x
                goal.pose.position.y = self.current_y
                goal.pose.position.z = self.current_z
                self.nav.goToPose(goal)     
                print(goal.pose.orientation.z)
                print("Turning left")
                self.right_turn = 1
                print("FRONT & LEFT DETECTED - TURNING RIGHT")

        #left_turn flag is 1
        #Turn left
        elif self.left_turn == 1:                
                # Get the robot's current orientation in radians
                current_orient_rad = math.atan2(2.0 * (self.current_orient_w * self.current_orient_z + self.current_orient_x * self.current_orient_y), 
                                                1.0 - 2.0 * (self.current_orient_y * self.current_orient_y + self.current_orient_z * self.current_orient_z))
                # Calculate the new orientation for the right turn
                new_orient_rad = current_orient_rad + math.radians(103)

                # Calculate the quaternion from the new orientation
                quaternion = tf_transformations.quaternion_from_euler(0, 0, new_orient_rad)
                # Set goal.pose.orientation.z and goal.pose.orientation.w with the new quaternion
                goal.pose.orientation.z = quaternion[2]
                goal.pose.orientation.w = quaternion[3]
                goal.pose.position.x = self.current_x
                goal.pose.position.y = self.current_y
                goal.pose.position.z = self.current_z
                self.nav.goToPose(goal)     
                print(goal.pose.orientation.z)
                print("Turning left")
                if(self.nav.isTaskComplete()):
                    self.left_turn = 0
                    self.move_forward = 1      

        #move_forward flag is 1
        #Move forward
        if self.move_forward ==1 and self.nav.isTaskComplete():
            print("Left Turn Completed ---------------------- Moving Forward")
            twist.linear.x = 0.3
            self.publisher_Twist.publish(twist)


        elif self.left_turn == 1 and self.move_forward == 1 and self.nav.isTaskComplete():
            print("I'm triggering this infinite stuck loop")
            self.move_forward = 0


        if self.left > 1.5 and self.nav.isTaskComplete():
            print("I may be also causing this issue")
            self.left_turn = 1

    

                

def main(args=None):
    rclpy.init(args=args)
    maze_solver_node = MazeSolverNode()
    maze_solver_node.create_timer(1.0,maze_solver_node.navigate)
    rclpy.spin(maze_solver_node)
    
    
    maze_solver_node.destroy_node()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
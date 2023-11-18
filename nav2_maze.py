#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose






class MazeSolverNode(Node):
    def __init__(self):
        super().__init__('maze_solver_node')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.publisher_Twist = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_Pose = self.create_publisher(PoseStamped, 'move_base_simple/goal', 10)
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

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
        self.set_axis = True



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

        self.avg_right = np.mean(sections[8] + sections[9] + sections[10])
        self.avg_left = np.mean(sections[2] + sections[3] + sections[4])
        self.avg_front = np.mean(sections[0] + sections[1] + sections[11])
        self.avg_back = np.mean(sections[5] + sections[6] + sections[7])
        

    def navigate(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'  # Adjust the frame_id based on your robot's configuration
        goal.header.stamp = self.nav.get_clock().now().to_msg()

        # print("Right side: ", self.avg_right)
        # print("Left side: ", self.avg_left)
        # print("Back side: ", self.avg_back)
        # print("Front side: ", self.avg_front)
        # print("Current x: ", self.current_x)
        # print("Current y: ", self.current_y)
        # print("Current z: ", self.current_z)
        # print("Current orient x: ", self.current_orient_x)
        # print("Current orient y: ", self.current_orient_y)
        # print("Current orient z: ", self.current_orient_z)
        # print("Current orient w: ", self.current_orient_w)

        if self.avg_front < 0.7:
                goal.pose.position.x = float(self.current_x)
                print("Current orient z: ", self.current_orient_z)
                print("Current orient w: ", self.current_orient_w)

                goal.pose.orientation.z = (self.current_orient_z + 0.5)
                goal.pose.orientation.w = (self.current_orient_w - 0.2)

                self.nav.goToPose(goal)     
                print(goal.pose.orientation.z)
                print(goal.pose.orientation.w)
                print("Turning Left")
                if self.set_axis == True:
                    self.set_axis = False
                else:
                    self.set_axis = True


        elif (self.avg_left >0.8 and self.avg_front >1) or (self.avg_right >0.8 and self.avg_front > 0.8) :
                
                print("Current x: ", self.current_x)
                print("Current y: ", self.current_y)
                if self.set_axis == True:
                    goal.pose.position.x = (self.current_x + 0.5)
                    goal.pose.position.y = (self.current_y + 0.0)
                    print(goal.pose.position.x)
                    print(goal.pose.position.y)
                    self.nav.goToPose(goal)     
                    print("Going straight")


                else:
                    goal.pose.position.x = self.current_x + 0.0
                    goal.pose.position.y = (self.current_y + 0.5)
                    print(goal.pose.position.x)
                    print(goal.pose.position.y)
                    self.nav.goToPose(goal)     
                    print("Going straight")
                    self.goal_reached = False
                    goal_msg = NavigateToPose.Goal()
                    goal_msg.pose = goal

        while not self.nav.isTaskComplete():
            
            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')

                     

    


            

def main(args=None):
    rclpy.init(args=args)
    maze_solver_node = MazeSolverNode()
    maze_solver_node.create_timer(1.0,maze_solver_node.navigate)
    rclpy.spin(maze_solver_node)
    
    
    maze_solver_node.destroy_node()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
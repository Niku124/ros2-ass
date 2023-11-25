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
        
        # A bunch of initializations stuff
        # Subscribed to Odom to check the current position of the robot
        # Subscribed to Laser to check the distance from the walls
        # Publisher for to send Twist commmands to the robot
        # Action Client to use Nav2

        super().__init__('maze_solver_node')
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.publisher_Twist = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_Pose = self.create_publisher(PoseStamped, 'move_base_simple/goal', 10)
        self._action_client = ActionClient(self, NavigateToPose, 'NavigateToPose')

        #Creates a BasicNavigator object to use Nav2
        self.nav = BasicNavigator()

        #Initializes all the variables, some of them are unused but i'm too lazy to remove them
        self.current_pose = None
        self.immediate_front = 0.0
        self.immediate_back = 0.0
        self.immediate_left = 0.0
        self.immediate_right = 0.0

        self.front = 0.0
        self.left = 0.0
        self.right = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_orient_x = 0.0
        self.current_orient_y = 0.0
        self.current_orient_z = 0.0
        self.current_orient_w = 0.0
        self.left_turn = 0
        self.right_turn = 0
        self.move_forward = 0

        self.is_turning = False

        self.move_forward_counter = 0
        

    #This is the odom callback function
    #Assigning the current position of the robot to the variables declared above
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        self.current_orient_x = msg.pose.pose.orientation.x
        self.current_orient_y = msg.pose.pose.orientation.y
        self.current_orient_z = msg.pose.pose.orientation.z
        self.current_orient_w = msg.pose.pose.orientation.w



    #This is the laser callback function
    # So basically i'm parsing the laser data into 12 sections to map the clockwise direction
    def laser_callback(self, msg):
        ranges = msg.ranges
        num_ranges = len(ranges)
        section = num_ranges // 12
        sections = [ranges[i * section:(i + 1) * section] for i in range(12)]

        self.immediate_right = np.mean(sections[9])
        self.immediate_left = np.mean(sections[3])
        self.immediate_front = np.mean(sections[0])
        self.immediate_back = np.mean(sections[6])

        self.current_2oclock = np.mean(sections[11])
        self.current_10oclock = np.mean(sections[2])

        #Total average of the 3 sections in each direction
        self.right = np.mean(sections[8] + sections[9] + sections[10])
        self.left = np.mean(sections[2] + sections[3] + sections[4])
        self.front = np.mean(sections[0] + sections[1] + sections[11])

    

    def navigate(self):
        
        twist = Twist()
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.nav.get_clock().now().to_msg()

        print("Left Turn Status : ", self.left_turn)
        print("Right Turn Status : ", self.right_turn)
        print("Move Forward Status : ", self.move_forward)


        print("Left : ", self.left)
        print("Average Left : ", self.immediate_left)


        #Check for the left wall first
        if self.left < 0.9 :
            self.left_wall_detected = True

            #Left wall detected, now checking the front to see if there's a wall
            #Expected Behaviour : Move Forward
            if self.front > 0.7:   
                twist.linear.x = 0.3
                twist.angular.z = 0.0
                self.publisher_Twist.publish(twist)
                self.right_turn = 0
                self.move_forward = 1
                print("LEFT DETECTED - MOVING FORWARD")

            #Left wall detected, Front wall also detected and right_turn flag is 0
            #Expected Behaviour : Turn Right
            elif self.front <0.7 and self.right_turn == 0:

                # Get the robot's current orientation in radians
                current_orient_rad = math.atan2(2.0 * (self.current_orient_w * self.current_orient_z + self.current_orient_x * self.current_orient_y),1.0 - 2.0 * (self.current_orient_y * self.current_orient_y + self.current_orient_z * self.current_orient_z))
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


        #Checks if the left turn flag is up and its not turning
        #Expected Behaviour : Turn Left
        elif self.left_turn == 1 and not self.is_turning and self.left:
        
                #Calcualtions to get the Quaternion values for rotation
                current_orientation = [self.current_orient_x, self.current_orient_y, self.current_orient_z, self.current_orient_w]
                roll, pitch, yaw = euler_from_quaternion(current_orientation)
                yaw += math.radians(103)
                if yaw < -math.pi:
                    yaw += 2 * math.pi
                elif yaw > math.pi:
                    yaw -= 2 * math.pi
                new_quaternion = quaternion_from_euler(roll, pitch, yaw)
                goal.pose.orientation.z = new_quaternion[2]
                goal.pose.orientation.w = new_quaternion[3]
                goal.pose.position.x = self.current_x
                goal.pose.position.y = self.current_y
                goal.pose.position.z = self.current_z
                self.is_turning = True
                self.nav.goToPose(goal)
                print(goal.pose.orientation.z,goal.pose.orientation.w)     
                print("Turning left")

        #Checks if it has turned left, then it moves forward 
        elif(self.left_turn == 1 and self.is_turning == True and self.nav.isTaskComplete()):
            self.left_turn = 0
            self.move_forward = 1
            self.is_turning = False
            print("WTFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFff")
                         
        #Move forward Function
        #Publishes Twist commands to move forward
        elif self.move_forward ==1:
            print("Left Turn Completed - Moving Forward")
            twist.linear.x = 0.3
            self.publisher_Twist.publish(twist)


        #Checks if it has turned left and has moved forward
        #Expected Behaviour : set move_flag to 0 
        elif self.left_turn == 1 and self.move_forward == 1:
            self.move_forward = 0
            print("I'm messing things uppppppppppppppppppppppppppppppppppppppppppppppppppppppppppppppppp")


        #Checks if the left side is not detected and its currently not turning
        #Expected Behaviour : Left_Turn flag up

        #This is the function that messes things up, play around with the self.left detection value
        if self.left > 1.6 and self.is_turning == False:
            self.left_turn = 1
            print("HUHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH")

                # In your main loop
        if self.move_forward == 1 and:
            self.move_forward_counter += 1
        else:
            self.move_forward_counter = 0


        if self.move_forward_counter > 5:
            self.move_forward = 0
            self.move_forward_counter = 0
            twist.linear.x = 0.0
            self.publisher_Twist.publish(twist)
            self.left_turn = 1
            print("Move forward reset")

        print(f"Move forward counter: {self.move_forward_counter}")

        if(self.front > 0.7):
            self.move_forward = 1
                

def main(args=None):
    rclpy.init(args=args)
    maze_solver_node = MazeSolverNode()
    maze_solver_node.create_timer(1.0,maze_solver_node.navigate)
    rclpy.spin(maze_solver_node)
    
    
    maze_solver_node.destroy_node()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
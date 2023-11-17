#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations


def create_pose_stamped(navigator: BasicNavigator, position_x,position_y,orientation_z):
    q_x,q_y,q_z,q_w = tf_transformations.quaternion_from_euler(0, 0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    return pose


def main():
    
    rclpy.init()
    nav = BasicNavigator()

    # Set initial pose
    q_x,q_y,q_z,q_w = tf_transformations.quaternion_from_euler(0, 0, 0)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.x = q_x
    initial_pose.pose.orientation.y = q_y
    initial_pose.pose.orientation.z = q_z
    initial_pose.pose.orientation.w = q_w
    nav.setInitialPose(initial_pose)


    nav.waitUntilNav2Active()

    # # send singular Nav2 goal
    # goal = PoseStamped()
    # goal.header.frame_id = 'map'
    # goal.header.stamp = nav.get_clock().now().to_msg()
    # goal.pose.position.x = 1.0 
    # goal.pose.position.y = 0.0
    # goal.pose.position.z = 0.0
    # goal.pose.orientation.x = q_x
    # goal.pose.orientation.y = q_y
    # goal.pose.orientation.z = q_z
    # goal.pose.orientation.w = q_w
    # nav.goToPose(goal)


    # Sending Nav2 Waypoints
    waypoints = []


    waypoints.append(create_pose_stamped(nav, 1.0, 0.0, 0.0))
    waypoints.append(create_pose_stamped(nav, 2.0, 0.0, 0.0))
    waypoints.append(create_pose_stamped(nav, 3.0, 0.0, 0.0))
    waypoints.append(create_pose_stamped(nav, 4.0, 0.0, 0.0))
    waypoints.append(create_pose_stamped(nav, 5.0, 0.0, 0.0))

    nav.followWaypoints(waypoints)


    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(nav.getResult())



    rclpy.shutdown()



if __name__ == '__main__':
    main()
#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()

    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -0.012046
    initial_pose.pose.position.y = 0.000007
    initial_pose.pose.orientation.z = 0.038716
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    # Go to our demos first goal pose
    goal_poses = []

    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 1.775028
    goal_pose1.pose.position.y = -1.789175
    goal_pose1.pose.orientation.z = 0.038716
    goal_poses.append(goal_pose1)

    # additional goals can be appended
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 2.179500
    goal_pose2.pose.position.y = 1.512253
    goal_pose2.pose.orientation.z = 0.038716
    goal_poses.append(goal_pose2)


    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = 1.088417
    goal_pose3.pose.position.y = 2.986520
    goal_pose3.pose.orientation.z = 0.038716
    goal_poses.append(goal_pose3)

    goal_pose4 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -0.043882
    goal_pose3.pose.position.y = 1.487983
    goal_pose3.pose.orientation.z = 0.038716
    goal_poses.append(goal_pose4)

    # Get the path, smooth it
    count = 0
    while(count < 4):
        #navigator.followPath(smoothed_path)
        navigator.goToPose(goal_poses[count])
        navigator.waitUntilNav2Active()
        
        i = 0
        # while not navigator.isTaskComplete():
        #     i += 1
        #     feedback = navigator.getFeedback()
        #     if feedback and i % 5 == 0:
        #         print('Estimated distance remaining to goal position: ' +
        #             '{0:.3f}'.format(feedback.distance_to_goal) +
        #             '\nCurrent speed of the robot: ' +
        #             '{0:.3f}'.format(feedback.speed))
        i = 0
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')
        count +=1

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)

if __name__ == '__main__':
    main()
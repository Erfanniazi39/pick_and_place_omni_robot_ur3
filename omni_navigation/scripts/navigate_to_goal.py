#!/usr/bin/env python3
"""
Script to navigate the robot to a specific goal using Nav2
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter
import sys
import time
from math import sin, cos


class RobotNavigator:
    def __init__(self):
        self.navigator = BasicNavigator()
        # Keep this client on simulation clock to match Nav2/Gazebo timestamps.
        self.navigator.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])

    def wait_until_ready(self, timeout_sec=60.0):
        """Wait until Nav2 lifecycle nodes and AMCL are active."""
        self.navigator.info('Waiting for Nav2 and AMCL to become active...')
        start = time.time()
        while time.time() - start < timeout_sec:
            try:
                # With localizer='amcl', this verifies both Nav2 and AMCL lifecycle state.
                self.navigator.waitUntilNav2Active(localizer='amcl')
                self.navigator.info('Nav2 and AMCL are active.')
                return True
            except Exception as exc:
                self.navigator.warn(f'Nav2 not ready yet: {exc}')
                time.sleep(1.0)

        self.navigator.error('Timed out waiting for Nav2/AMCL activation.')
        return False

    def navigate_to_goal(self, x, y, theta=0.0):
        """
        Navigate robot to specified goal coordinates
        
        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            theta: Rotation in radians (0.0 = forward facing)
        """
        self.navigator.info(f'Starting navigation to goal: ({x}, {y}, {theta})')

        if not self.wait_until_ready(timeout_sec=60.0):
            return False

        self.navigator.info(
            'Ensure initial pose has been set in RViz (2D Pose Estimate) before sending goal.'
        )
        
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        # Use latest available TF instead of a specific timestamp to avoid
        # "extrapolation into the future" rejections in simulation.
        goal_pose.header.stamp.sec = 0
        goal_pose.header.stamp.nanosec = 0
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        # Convert Euler angles to quaternion (roll=0, pitch=0, yaw=theta)
        half_theta = theta / 2.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = sin(half_theta)
        goal_pose.pose.orientation.w = cos(half_theta)
        
        # Send goal
        accepted = self.navigator.goToPose(goal_pose)
        if not accepted:
            self.navigator.error(
                'Goal was rejected by Nav2. Set initial pose in RViz and ensure goal is in free map space.'
            )
            return False
        
        # Monitor navigation
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            
            if feedback and i % 5 == 0:
                self.navigator.info(
                    f'Distance remaining: {feedback.distance_remaining:.2f} meters'
                )
        
        # Check final result
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.navigator.info('Goal reached successfully')
            return True
        elif result == TaskResult.CANCELED:
            self.navigator.warn('Navigation was canceled')
            return False
        else:
            self.navigator.error('Navigation failed')
            return False


def main():
    rclpy.init()
    
    # Parse command line arguments
    if len(sys.argv) > 2:
        goal_x = float(sys.argv[1])
        goal_y = float(sys.argv[2])
        goal_theta = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    else:
        # Default: navigate to the red cube location
        goal_x = -14.0
        goal_y = 4.5
        goal_theta = 0.0
        print(f"No arguments provided. Using default: ({goal_x}, {goal_y}, {goal_theta})")
    
    navigator = RobotNavigator()
    
    print(f"\nNavigating robot to ({goal_x}, {goal_y}, {goal_theta})...")
    print("Make sure:")
    print("  1. Nav2 is running (ros2 launch omni_navigation all_in_one.launch.py)")
    print("  2. Initial pose is set in RViz (2D Pose Estimate tool)")
    print("  3. Map is loaded\n")
    
    success = navigator.navigate_to_goal(goal_x, goal_y, goal_theta)
    
    navigator.navigator.destroy_node()
    rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())

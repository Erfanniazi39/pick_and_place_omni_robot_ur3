#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time
from math import pi

class UR3GripperTester(Node):
    def __init__(self):
        super().__init__('ur3_gripper_tester')
        
        # Action clients for trajectory control
        self.ur3_client = ActionClient(self, FollowJointTrajectory, '/ur3_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        
        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.joint_states = {}
        self.get_logger().info('UR3 Gripper Tester initialized')
    
    def joint_state_callback(self, msg):
        """Store joint states for monitoring"""
        for name, position in zip(msg.name, msg.position):
            self.joint_states[name] = position
    
    def wait_for_servers(self, timeout=10):
        """Wait for action servers to be available"""
        self.get_logger().info('Waiting for UR3 controller...')
        if not self.ur3_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('UR3 controller not available!')
            return False
        
        self.get_logger().info('Waiting for Gripper controller...')
        if not self.gripper_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('Gripper controller not available!')
            return False
        
        self.get_logger().info('All controllers ready!')
        return True
    
    def move_ur3_home(self):
        """Move UR3 to home position"""
        self.get_logger().info('Moving UR3 to HOME position...')
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [
            'ur_shoulder_pan_joint',
            'ur_shoulder_lift_joint',
            'ur_elbow_joint',
            'ur_wrist_1_joint',
            'ur_wrist_2_joint',
            'ur_wrist_3_joint'
        ]
        
        # Home position (all zeros)
        point = JointTrajectoryPoint()
        point.positions = [0.0, -pi/2, pi/2, -pi/2, 0.0, 0.0]
        point.time_from_start.sec = 5
        goal.trajectory.points.append(point)
        
        future = self.ur3_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def move_ur3_ready(self):
        """Move UR3 to ready position (arm raised)"""
        self.get_logger().info('Moving UR3 to READY position...')
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [
            'ur_shoulder_pan_joint',
            'ur_shoulder_lift_joint',
            'ur_elbow_joint',
            'ur_wrist_1_joint',
            'ur_wrist_2_joint',
            'ur_wrist_3_joint'
        ]
        
        # Ready position
        point = JointTrajectoryPoint()
        point.positions = [0.0, -pi/3, pi/3, -pi/3, pi/2, 0.0]
        point.time_from_start.sec = 3
        goal.trajectory.points.append(point)
        
        future = self.ur3_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.5)
        return future.result()
    
    def gripper_open(self):
        """Open gripper (fingers fully extended)"""
        self.get_logger().info('Opening GRIPPER...')
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [
            'gripper_left_finger_joint',
            'gripper_right_finger_joint'
        ]
        
        # Open position (fully extended)
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]  # Both at 0 (open)
        point.time_from_start.sec = 2
        goal.trajectory.points.append(point)
        
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.5)
        
        left_pos = self.joint_states.get('gripper_left_finger_joint', None)
        right_pos = self.joint_states.get('gripper_right_finger_joint', None)
        self.get_logger().info(f'Gripper positions - Left: {left_pos}, Right: {right_pos}')
        
        return future.result()
    
    def gripper_close(self):
        """Close gripper (fingers fully closed)"""
        self.get_logger().info('Closing GRIPPER...')
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [
            'gripper_left_finger_joint',
            'gripper_right_finger_joint'
        ]
        
        # Closed position (fully retracted)
        point = JointTrajectoryPoint()
        point.positions = [-0.04, -0.04]  # Both at -0.04 (closed)
        point.time_from_start.sec = 2
        goal.trajectory.points.append(point)
        
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.5)
        
        left_pos = self.joint_states.get('gripper_left_finger_joint', None)
        right_pos = self.joint_states.get('gripper_right_finger_joint', None)
        self.get_logger().info(f'Gripper positions - Left: {left_pos}, Right: {right_pos}')
        
        return future.result()
    
    def gripper_partial(self):
        """Partially close gripper (50% closure)"""
        self.get_logger().info('Partially closing GRIPPER (50%)...')
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [
            'gripper_left_finger_joint',
            'gripper_right_finger_joint'
        ]
        
        # Partial position (50% closure)
        point = JointTrajectoryPoint()
        point.positions = [-0.02, -0.02]
        point.time_from_start.sec = 2
        goal.trajectory.points.append(point)
        
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.5)
        
        left_pos = self.joint_states.get('gripper_left_finger_joint', None)
        right_pos = self.joint_states.get('gripper_right_finger_joint', None)
        self.get_logger().info(f'Gripper positions - Left: {left_pos}, Right: {right_pos}')
        
        return future.result()
    
    def test_gripper(self):
        """Test gripper functionality"""
        self.get_logger().info('\n========== GRIPPER TEST START ==========')
        
        # Test 1: Open
        self.gripper_open()
        time.sleep(1)
        
        # Test 2: Close
        self.gripper_close()
        time.sleep(1)
        
        # Test 3: Partial
        self.gripper_partial()
        time.sleep(1)
        
        # Test 4: Open again
        self.gripper_open()
        time.sleep(1)
        
        self.get_logger().info('========== GRIPPER TEST COMPLETE ==========\n')
    
    def run_sequence(self):
        """Run complete UR3 + Gripper test sequence"""
        try:
            if not self.wait_for_servers():
                return
            
            self.get_logger().info('\n========== STARTING UR3 + GRIPPER TEST SEQUENCE ==========\n')
            
            # Step 1: Move to home
            self.move_ur3_home()
            time.sleep(2)
            
            # Step 2: Open gripper
            self.gripper_open()
            time.sleep(1)
            
            # Step 3: Move to ready position
            self.move_ur3_ready()
            time.sleep(1)
            
            # Step 4: Test gripper operations
            self.test_gripper()
            time.sleep(1)
            
            # Step 5: Return to home
            self.move_ur3_home()
            self.gripper_open()
            
            self.get_logger().info('\n========== TEST SEQUENCE COMPLETE ==========\n')
            
            # Summary
            self.print_joint_states()
            
        except Exception as e:
            self.get_logger().error(f'Error during test sequence: {e}')
    
    def print_joint_states(self):
        """Print current joint states"""
        self.get_logger().info('\n===== CURRENT JOINT STATES =====')
        for joint_name, position in sorted(self.joint_states.items()):
            self.get_logger().info(f'{joint_name}: {position:.4f}')


def main(args=None):
    rclpy.init(args=args)
    node = UR3GripperTester()
    
    try:
        node.run_sequence()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

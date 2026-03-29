#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time
from math import pi

class UR3GripperController(Node):
    def __init__(self):
        super().__init__('ur3_gripper_controller')
        
        # Action clients
        self.ur3_client = ActionClient(self, FollowJointTrajectory, '/ur3_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        
        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.joint_states = {}
        self.get_logger().info('UR3 Gripper Controller initialized')
    
    def joint_state_callback(self, msg):
        """Store joint states"""
        for name, position in zip(msg.name, msg.position):
            self.joint_states[name] = position
    
    def wait_for_servers(self, timeout=10):
        """Wait for action servers"""
        self.get_logger().info('Waiting for controllers...')
        ur3_ready = self.ur3_client.wait_for_server(timeout_sec=timeout)
        gripper_ready = self.gripper_client.wait_for_server(timeout_sec=timeout)
        
        if ur3_ready and gripper_ready:
            self.get_logger().info('All controllers ready!')
            return True
        else:
            self.get_logger().error('Controllers not ready!')
            return False
    
    def send_ur3_goal(self, joint_positions, time_seconds=3):
        """Send goal to UR3"""
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
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = time_seconds
        goal.trajectory.points.append(point)
        
        future = self.ur3_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_gripper_goal(self, left_pos, right_pos, time_seconds=2):
        """Send goal to gripper"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [
            'gripper_left_finger_joint',
            'gripper_right_finger_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = [left_pos, right_pos]
        point.time_from_start.sec = time_seconds
        goal.trajectory.points.append(point)
        
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.5)
        
        return future.result()
    
    def print_usage(self):
        """Print available commands"""
        print("\n" + "="*50)
        print("UR3 & GRIPPER CONTROL - Interactive Menu")
        print("="*50)
        print("\nARM COMMANDS:")
        print("  1 - Move to HOME (0, -π/2, π/2, -π/2, 0, 0)")
        print("  2 - Move to READY (arm raise)")
        print("  3 - Move to CUSTOM (enter positions)")
        print("\nGRIPPER COMMANDS:")
        print("  4 - OPEN gripper")
        print("  5 - CLOSE gripper")
        print("  6 - PARTIAL close (50%)")
        print("\nDISAGNOSTIC:")
        print("  7 - Show joint states")
        print("  8 - Run auto test sequence")
        print("  0 - EXIT")
        print("-"*50 + "\n")
    
    def get_joint_input(self):
        """Get 6 joint positions from user"""
        positions = []
        joints = [
            'shoulder_pan',
            'shoulder_lift',
            'elbow',
            'wrist_1',
            'wrist_2',
            'wrist_3'
        ]
        
        print("Enter joint positions in radians (-π to π):")
        for joint in joints:
            while True:
                try:
                    val = float(input(f"  {joint}: "))
                    if -pi <= val <= pi:
                        positions.append(val)
                        break
                    else:
                        print(f"  Please enter value between -{pi:.2f} and {pi:.2f}")
                except ValueError:
                    print("  Invalid input. Please enter a number.")
        
        return positions
    
    def run_interactive(self):
        """Run interactive control loop"""
        if not self.wait_for_servers():
            return
        
        while True:
            self.print_usage()
            choice = input("Enter command (0-8): ").strip()
            
            try:
                if choice == '0':
                    self.get_logger().info('Exiting...')
                    break
                
                elif choice == '1':
                    self.get_logger().info('Moving to HOME')
                    self.send_ur3_goal([0.0, -pi/2, pi/2, -pi/2, 0.0, 0.0], 5)
                    time.sleep(1)
                
                elif choice == '2':
                    self.get_logger().info('Moving to READY')
                    self.send_ur3_goal([0.0, -pi/3, pi/3, -pi/3, pi/2, 0.0], 3)
                    time.sleep(1)
                
                elif choice == '3':
                    positions = self.get_joint_input()
                    time_sec = float(input("Time to reach (seconds): "))
                    self.get_logger().info('Moving to custom position')
                    self.send_ur3_goal(positions, time_sec)
                    time.sleep(1)
                
                elif choice == '4':
                    self.get_logger().info('Opening gripper')
                    self.send_gripper_goal(0.0, 0.0, 2)
                    left = self.joint_states.get('gripper_left_finger_joint', 'N/A')
                    right = self.joint_states.get('gripper_right_finger_joint', 'N/A')
                    self.get_logger().info(f'Gripper: Left={left}, Right={right}')
                
                elif choice == '5':
                    self.get_logger().info('Closing gripper')
                    self.send_gripper_goal(-0.04, -0.04, 2)
                    left = self.joint_states.get('gripper_left_finger_joint', 'N/A')
                    right = self.joint_states.get('gripper_right_finger_joint', 'N/A')
                    self.get_logger().info(f'Gripper: Left={left}, Right={right}')
                
                elif choice == '6':
                    self.get_logger().info('Partial gripper close (50%)')
                    self.send_gripper_goal(-0.02, -0.02, 2)
                    left = self.joint_states.get('gripper_left_finger_joint', 'N/A')
                    right = self.joint_states.get('gripper_right_finger_joint', 'N/A')
                    self.get_logger().info(f'Gripper: Left={left}, Right={right}')
                
                elif choice == '7':
                    print("\n===== JOINT STATES =====")
                    for name, pos in sorted(self.joint_states.items()):
                        print(f"{name}: {pos:.4f}")
                    print("========================\n")
                
                elif choice == '8':
                    self.run_auto_test()
                
                else:
                    print("Invalid command!")
            
            except Exception as e:
                self.get_logger().error(f'Error: {e}')
    
    def run_auto_test(self):
        """Automatic test sequence"""
        self.get_logger().info('\n========== AUTO TEST SEQUENCE ==========\n')
        
        commands = [
            ('HOME', [0.0, -pi/2, pi/2, -pi/2, 0.0, 0.0], None, 5),
            ('READY', [0.0, -pi/3, pi/3, -pi/3, pi/2, 0.0], None, 3),
            ('Open Gripper', None, (0.0, 0.0), 2),
            ('Close Gripper', None, (-0.04, -0.04), 2),
            ('Partial Grip', None, (-0.02, -0.02), 2),
            ('Open', None, (0.0, 0.0), 2),
            ('HOME', [0.0, -pi/2, pi/2, -pi/2, 0.0, 0.0], None, 5),
        ]
        
        for cmd_name, ur3_pos, grip_pos, duration in commands:
            self.get_logger().info(f'Executing: {cmd_name}')
            
            if ur3_pos is not None:
                self.send_ur3_goal(ur3_pos, duration)
            elif grip_pos is not None:
                self.send_gripper_goal(grip_pos[0], grip_pos[1], duration)
            
            time.sleep(duration + 0.5)
        
        self.get_logger().info('\n========== AUTO TEST COMPLETE ==========\n')


def main(args=None):
    rclpy.init(args=args)
    node = UR3GripperController()
    
    try:
        node.run_interactive()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

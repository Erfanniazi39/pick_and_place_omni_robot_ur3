#!/usr/bin/env python3

import os
import subprocess
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from ament_index_python.packages import get_package_share_directory
import math
import time

try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False


def load_yaml(path):
    with open(path, 'r', encoding='utf-8') as file:
        return yaml.safe_load(file)


class UR3MotionPlanner(Node):
    def __init__(self):
        super().__init__('ur3_motion_planner')
        self._joint_state_received = False
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
        )
        
        if not MOVEIT_AVAILABLE:
            self.get_logger().error('MoveIt2 Python API not available! Install moveit_py')
            return
        
        # Initialize MoveIt2 using Jazzy MoveItPy API (config_dict-based)
        self.moveit = MoveItPy(
            node_name='ur3_motion_planner_moveit',
            config_dict=self._build_moveit_config(),
        )
        self.ur3_arm = self.moveit.get_planning_component("ur3_arm")
        self.gripper = self.moveit.get_planning_component("gripper")
        self.arm_with_gripper = self.moveit.get_planning_component("arm_with_gripper")

        self._wait_for_joint_states()

        self.get_logger().info('UR3 Motion Planner initialized')
        self.get_logger().info(
            f'Available planning groups: {self.moveit.get_robot_model().joint_model_group_names}'
        )

    def _build_moveit_config(self):
        """Build MoveIt config dict for MoveItPy constructor."""
        omni_moveit2_dir = get_package_share_directory('omni_moveit2')
        omni_description_dir = get_package_share_directory('omni_description')

        robot_description_xml = subprocess.check_output(
            ['xacro', os.path.join(omni_description_dir, 'urdf', 'omni_robot.urdf.xacro')],
            text=True,
        )
        robot_description = {'robot_description': robot_description_xml}

        robot_description_semantic = {
            'robot_description_semantic': open(
                os.path.join(omni_moveit2_dir, 'config', 'omni_robot.srdf'),
                'r',
                encoding='utf-8',
            ).read()
        }

        robot_description_kinematics = {
            'robot_description_kinematics': load_yaml(
                os.path.join(omni_moveit2_dir, 'config', 'kinematics.yaml')
            )
        }

        ompl_config = {
            'ompl': load_yaml(
                os.path.join(omni_moveit2_dir, 'config', 'moveit_planning.yaml')
            )
        }

        moveit_controllers = load_yaml(
            os.path.join(omni_moveit2_dir, 'config', 'moveit_controllers.yaml')
        )

        planning_pipeline_config = {
            'planning_pipelines': {
                'pipeline_names': ['ompl'],
                'default_planning_pipeline': 'ompl',
            }
        }

        trajectory_execution = {
            'trajectory_execution': {
                'allowed_execution_duration_scaling': 1.5,
                'allowed_goal_duration_margin': 0.75,
                'moveit_manage_controllers': False,
            }
        }

        plan_request_params = {
            'plan_request_params': {
                'planner_id': 'RRTConnectkConfigDefault',
                'planning_pipeline': 'ompl',
                'planning_time': 15.0,
                'planning_attempts': 1,
                'max_velocity_scaling_factor': 1.0,
                'max_acceleration_scaling_factor': 1.0,
            }
        }

        planning_scene_monitor_parameters = {
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
            'publish_robot_description': True,
            'publish_robot_description_semantic': True,
        }

        config = {}
        for block in [
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline_config,
            ompl_config,
            moveit_controllers,
            trajectory_execution,
            plan_request_params,
            planning_scene_monitor_parameters,
        ]:
            config.update(block)
        return config
    
    def plan_to_joint_state(self, group_name, joint_positions, max_time=5.0):
        """Plan arm to joint configuration"""
        self.get_logger().info(f'Planning {group_name} to positions: {[f"{p:.2f}" for p in joint_positions]}')

        planning_component = self.moveit.get_planning_component(group_name)

        try:
            robot_state = RobotState(self.moveit.get_robot_model())
            robot_state.set_joint_group_positions(group_name, joint_positions)

            planning_component.set_start_state_to_current_state()
            planning_component.set_goal_state(robot_state=robot_state)

            plan_result = planning_component.plan()

            if plan_result:
                self.get_logger().info('✓ Motion plan found!')
                if hasattr(plan_result, 'trajectory') and hasattr(plan_result.trajectory, 'joint_trajectory'):
                    self.get_logger().info(
                        f'  Waypoints: {len(plan_result.trajectory.joint_trajectory.points)}'
                    )
                return plan_result

            self.get_logger().error('✗ Motion planning failed!')
            return None

        except Exception as e:
            self.get_logger().error(f'Planning error: {e}')
            return None

    def _joint_state_callback(self, msg):
        if msg.name:
            self._joint_state_received = True

    def _wait_for_joint_states(self, timeout_sec=5.0):
        start = time.time()
        while time.time() - start < timeout_sec and rclpy.ok() and not self._joint_state_received:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._joint_state_received:
            self.get_logger().info('Received initial joint states')
        else:
            self.get_logger().warn('No joint states received before planning start')
    
    def plan_to_pose(self, target_pose, max_time=5.0):
        """Plan arm to end-effector pose"""
        self.get_logger().info(f'Planning to pose: position=[{target_pose.position.x:.2f}, {target_pose.position.y:.2f}, {target_pose.position.z:.2f}]')
        
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "world"
            pose_stamped.pose = target_pose
            
            # Plan motion
            plan_result = self.ur3_arm.plan(
                pose_goal=pose_stamped,
                frame_id="world",
                max_time=max_time
            )
            
            if plan_result:
                self.get_logger().info('✓ Motion plan found!')
                self.get_logger().info(f'  Waypoints: {len(plan_result.trajectory.joint_trajectory.points)}')
                return plan_result.trajectory.joint_trajectory
            else:
                self.get_logger().error('✗ Motion planning failed!')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Planning error: {e}')
            return None
    
    def execute_trajectory(self, plan_result):
        """Execute planned trajectory using MoveItPy trajectory execution manager"""
        self.get_logger().info('Executing trajectory...')

        try:
            if hasattr(plan_result, 'trajectory'):
                self.moveit.execute(plan_result.trajectory)
                self.get_logger().info('✓ Trajectory execution started')
                return True

            self.get_logger().error('Execution error: plan result has no trajectory')
            return False
        except Exception as e:
            self.get_logger().error(f'Execution error: {e}')
            return False
    
    def test_motion_planning(self):
        """Test motion planning capabilities"""
        self.get_logger().info('\n========== MOTION PLANNING TEST ==========\n')
        
        # Test 1: Plan to home position
        self.get_logger().info('Test 1: UR3 ARM to HOME')
        joint_goal = [0.0, -math.pi/2, math.pi/2, -math.pi/2, 0.0, 0.0]
        plan_result = self.plan_to_joint_state('ur3_arm', joint_goal)

        if plan_result:
            self.get_logger().info('  ✓ Can reach home position')
            if hasattr(plan_result, 'trajectory') and hasattr(plan_result.trajectory, 'joint_trajectory'):
                for i, point in enumerate(plan_result.trajectory.joint_trajectory.points[:3]):
                    self.get_logger().info(f'    Waypoint {i}: {[f"{p:.3f}" for p in point.positions]}')
        
        # Test 2: Plan to ready position
        self.get_logger().info('\nTest 2: UR3 ARM to READY')
        joint_goal = [0.0, -math.pi/3, math.pi/3, -math.pi/3, math.pi/2, 0.0]
        plan_result = self.plan_to_joint_state('ur3_arm', joint_goal, max_time=10.0)

        if plan_result:
            self.get_logger().info('  ✓ Can reach ready position')
        
        # Test 3: Gripper open
        self.get_logger().info('\nTest 3: GRIPPER OPEN')
        gripper_goal = [0.0, 0.0]
        plan_result = self.plan_to_joint_state('gripper', gripper_goal)

        if plan_result:
            self.get_logger().info('  ✓ Gripper can open')
        
        # Test 4: Gripper closed
        self.get_logger().info('\nTest 4: GRIPPER CLOSE')
        gripper_goal = [-0.04, -0.04]
        plan_result = self.plan_to_joint_state('gripper', gripper_goal)

        if plan_result:
            self.get_logger().info('  ✓ Gripper can close')
        
        self.get_logger().info('\n========== TEST COMPLETE ==========\n')
    
    def print_system_info(self):
        """Print system information"""
        self.get_logger().info('\n========== SYSTEM INFORMATION ==========')
        
        groups = list(self.moveit.get_robot_model().joint_model_group_names)
        self.get_logger().info(f'Planning groups: {groups}')

        for group_name in groups:
            try:
                group = self.moveit.get_robot_model().get_joint_model_group(group_name)
                self.get_logger().info(f'\n  {group_name}:')
                self.get_logger().info(f'    Links: {group.link_model_names}')
            except Exception as e:
                self.get_logger().warning(f'  Could not get info for {group_name}: {e}')
        
        self.get_logger().info('\n========== END INFO ==========\n')


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = UR3MotionPlanner()

        if MOVEIT_AVAILABLE:
            node.print_system_info()
            node.test_motion_planning()
        else:
            node.get_logger().error('Cannot run tests without MoveIt2')

    except KeyboardInterrupt:
        pass
    finally:
        if node is not None and MOVEIT_AVAILABLE and hasattr(node, 'moveit'):
            try:
                node.moveit.shutdown()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()

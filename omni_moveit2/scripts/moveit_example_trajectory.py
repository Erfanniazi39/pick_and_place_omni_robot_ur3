#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time
import math
import yaml
import subprocess
import os
from pathlib import Path

try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False


class UR3ExampleTrajectory(Node):
    def __init__(self):
        super().__init__('ur3_example_trajectory')
        self._joint_state_received = False
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
        )
        
        if not MOVEIT_AVAILABLE:
            self.get_logger().error('MoveIt2 not available!')
            return
        
        # Initialize MoveIt2 with new API
        self.moveit = MoveItPy(node_name='ur3_example_trajectory_moveit', config_dict=self._build_moveit_config())
        self.ur3_arm = self.moveit.get_planning_component("ur3_arm")
        self.gripper = self.moveit.get_planning_component("gripper")
        self._wait_for_joint_states()
        
        # Action client
        self.ur3_client = ActionClient(
            self, FollowJointTrajectory, '/ur3_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Example Trajectory Runner initialized')

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
    
    def _build_moveit_config(self):
        """Build MoveIt2 configuration dictionary from YAML files and xacro"""
        import subprocess
        from pathlib import Path
        
        ws_path = Path.home() / 'ros2_ws' / 'src'
        omni_moveit2_dir = str(ws_path / 'omni_moveit2')
        
        def load_yaml_file(path):
            with open(path, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        
        # Generate robot description from xacro
        xacro_path = ws_path / 'omni_description' / 'urdf' / 'omni_robot.urdf.xacro'
        urdf_text = subprocess.run(
            ['xacro', str(xacro_path)],
            capture_output=True, text=True, timeout=5
        ).stdout
        
        robot_description = {'robot_description': urdf_text}
        
        robot_description_semantic = {
            'robot_description_semantic': open(
                os.path.join(omni_moveit2_dir, 'config', 'omni_robot.srdf'),
                'r', encoding='utf-8',
            ).read()
        }
        
        robot_description_kinematics = {
            'robot_description_kinematics': load_yaml_file(
                os.path.join(omni_moveit2_dir, 'config', 'kinematics.yaml')
            )
        }
        
        ompl_config = {
            'ompl': load_yaml_file(
                os.path.join(omni_moveit2_dir, 'config', 'moveit_planning.yaml')
            )
        }
        
        moveit_controllers = load_yaml_file(
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
    
    def wait_for_servers(self):
        """Wait for action servers"""
        if not self.ur3_client.wait_for_server(timeout_sec=5):
            self.get_logger().error('UR3 controller unavailable')
            return False
        if not self.gripper_client.wait_for_server(timeout_sec=5):
            self.get_logger().error('Gripper controller unavailable')
            return False
        return True
    
    def execute_trajectory(self, trajectory, target_name):
        """Execute a trajectory"""
        self.get_logger().info(f'Executing: {target_name}')

        self._ensure_strictly_increasing_timestamps(trajectory)
        
        if 'gripper' in target_name.lower():
            client = self.gripper_client
        else:
            client = self.ur3_client
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        try:
            send_goal_future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()

            if goal_handle is None:
                self.get_logger().error(f'Execution failed: no goal handle returned for {target_name}')
                return False

            if not goal_handle.accepted:
                self.get_logger().error(f'Execution rejected by controller: {target_name}')
                return False

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result_wrap = result_future.result()

            if result_wrap is None:
                self.get_logger().error(f'Execution failed: no result returned for {target_name}')
                return False

            error_code = result_wrap.result.error_code
            if error_code == 0:
                self.get_logger().info(f'✓ {target_name} complete')
                return True

            self.get_logger().error(
                f'Execution failed for {target_name} with controller error_code={error_code}'
            )
            return False
        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}')
            return False

    def _duration_to_seconds(self, duration_msg):
        return float(duration_msg.sec) + float(duration_msg.nanosec) * 1e-9

    def _set_duration_seconds(self, duration_msg, seconds):
        sec = int(seconds)
        nanosec = int((seconds - sec) * 1e9)
        duration_msg.sec = sec
        duration_msg.nanosec = nanosec

    def _ensure_strictly_increasing_timestamps(self, trajectory):
        """Controllers reject trajectories if waypoint times are equal or decreasing."""
        if not trajectory.points:
            return

        # If points have no valid timing (common when time parameterization is disabled),
        # assign a simple monotonic profile.
        first = self._duration_to_seconds(trajectory.points[0].time_from_start)
        needs_retime = first <= 0.0
        prev = first
        for pt in trajectory.points[1:]:
            cur = self._duration_to_seconds(pt.time_from_start)
            if cur <= prev:
                needs_retime = True
                break
            prev = cur

        if not needs_retime:
            return

        dt = 0.2
        for i, pt in enumerate(trajectory.points):
            self._set_duration_seconds(pt.time_from_start, i * dt)

        # Ensure strict increase from point 0 to 1 for controllers that validate this.
        if len(trajectory.points) > 1 and self._duration_to_seconds(trajectory.points[1].time_from_start) <= 0.0:
            self._set_duration_seconds(trajectory.points[1].time_from_start, dt)

        self.get_logger().warning('Trajectory had invalid timing; applied fallback monotonic timestamps')
    
    def plan_and_execute(self, group_name, joint_positions, description):
        """Plan and execute to joint positions"""
        self.get_logger().info(f'\n--- {description} ---')
        
        planning_component = self.moveit.get_planning_component(group_name)
        
        try:
            # Build goal state using RobotState
            robot_state = RobotState(self.moveit.get_robot_model())
            robot_state.set_joint_group_positions(group_name, joint_positions)
            
            # Set start to current state and goal to target
            planning_component.set_start_state_to_current_state()
            planning_component.set_goal_state(robot_state=robot_state)
            
            # Execute planning
            plan_result = planning_component.plan()
            
            if plan_result:
                # Jazzy MoveItPy returns moveit.core.robot_trajectory.RobotTrajectory.
                # Convert it to message type before accessing joint_trajectory.
                robot_traj_msg = plan_result.trajectory.get_robot_trajectory_msg()
                joint_traj = robot_traj_msg.joint_trajectory
                self.get_logger().info(f'Plan: {len(joint_traj.points)} waypoints')
                return self.execute_trajectory(
                    joint_traj,
                    description
                )
            else:
                self.get_logger().error('Planning failed')
                return False
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            return False
    
    def run_pick_and_place_example(self):
        """Example: Pick and place workflow"""
        self.get_logger().info('\n========== PICK AND PLACE EXAMPLE ==========\n')
        
        try:
            # Step 1: Move to home
            self.plan_and_execute(
                'ur3_arm',
                [0.0, -math.pi/2, math.pi/2, -math.pi/2, 0.0, 0.0],
                'Move to HOME'
            )
            time.sleep(1)
            
            # Step 2: Open gripper
            self.plan_and_execute(
                'gripper',
                [0.0, 0.0],
                'Gripper OPEN'
            )
            time.sleep(1)
            
            # Step 3: Move to approach object
            self.plan_and_execute(
                'ur3_arm',
                [0.5, -math.pi/4, math.pi/4, -math.pi/4, math.pi/2, 0.0],
                'Move to APPROACH position'
            )
            time.sleep(1)
            
            # Step 4: Close gripper
            self.plan_and_execute(
                'gripper',
                [-0.04, -0.04],
                'Gripper CLOSE'
            )
            time.sleep(1)
            
            # Step 5: Lift object
            self.plan_and_execute(
                'ur3_arm',
                [0.5, -math.pi/3, math.pi/3, -math.pi/3, math.pi/2, 0.0],
                'LIFT object'
            )
            time.sleep(1)
            
            # Step 6: Move to place location
            self.plan_and_execute(
                'ur3_arm',
                [-0.5, -math.pi/3, math.pi/3, -math.pi/3, math.pi/2, 0.0],
                'Move to PLACE position'
            )
            time.sleep(1)
            
            # Step 7: Open gripper
            self.plan_and_execute(
                'gripper',
                [0.0, 0.0],
                'Gripper OPEN (release)'
            )
            time.sleep(1)
            
            # Step 8: Return home
            self.plan_and_execute(
                'ur3_arm',
                [0.0, -math.pi/2, math.pi/2, -math.pi/2, 0.0, 0.0],
                'Return to HOME'
            )
            
            self.get_logger().info('\n========== PICK AND PLACE COMPLETE ==========\n')
            
        except Exception as e:
            self.get_logger().error(f'Example failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = UR3ExampleTrajectory()
        
        if MOVEIT_AVAILABLE and node.wait_for_servers():
            node.run_pick_and_place_example()
        else:
            node.get_logger().error('Cannot run example')
    
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Interrupted')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

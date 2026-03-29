#!/usr/bin/env python3

import math
import os
import time
import yaml
import subprocess

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False


class PickMiddleCube(Node):
    def __init__(self):
        super().__init__('pick_middle_cube')

        self._joint_state_received = False
        self._latest_joint_positions = {}
        self._holding_object = False
        self._joint_state_topics = set()
        self._joint_state_subscriptions = []
        default_joint_topics = list(self.declare_parameter('joint_state_topics', ['/joint_states']).value)
        for topic_name in default_joint_topics:
            self._subscribe_joint_state_topic(str(topic_name))
        self._discover_and_subscribe_joint_state_topics()

        # Subscribe to small cube color and big-cube target, plus odometry for navigation
        small_color_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.small_color_sub = self.create_subscription(
            String,
            '/color_match/small_cube_color',
            self._small_color_cb,
            small_color_qos,
        )
        self.big_target_sub = self.create_subscription(
            String,
            '/color_match/target_big_cube',
            self._big_target_cb,
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        picked_color_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.picked_color_pub = self.create_publisher(String, '/color_match/picked_cube_color', picked_color_qos)
        self.scan_start_pub = self.create_publisher(Bool, '/color_match/start_scan', picked_color_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 20)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.small_cube_color = None
        self.big_cube_target = None
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.ur3_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/ur3_controller/follow_joint_trajectory',
        )
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
        )

        if not MOVEIT_AVAILABLE:
            self.get_logger().error('MoveIt2 Python API not available. Install moveit_py and MoveIt2 packages.')
            return

        self.moveit = MoveItPy(
            node_name='pick_middle_cube_moveit',
            config_dict=self._build_moveit_config(),
        )

        self.home_pose = [0.0, -1.57, 1.57, -1.57, 0.0, 0.0]
        self.pre_grasp_pose = [0.0, -1.42, 2.00, -2.10, -1.57, 0.0]
        self.grasp_pose = [0.0, -1.58, 2.24, -2.23, -1.57, 0.0]
        self.lift_pose = [0.0, -1.25, 1.78, -2.05, -1.57, 0.0]
        self.gripper_open_pos = float(self.declare_parameter('gripper_open_pos', 0.015).value)
        self.gripper_close_pos = float(self.declare_parameter('gripper_close_pos', -0.075).value)
        self.gripper_squeeze_delta = float(self.declare_parameter('gripper_squeeze_delta', -0.010).value)
        self.enable_gripper_squeeze = bool(self.declare_parameter('enable_gripper_squeeze', True).value)
        self.gripper_min_limit = float(self.declare_parameter('gripper_min_limit', -0.07).value)
        self.gripper_max_limit = float(self.declare_parameter('gripper_max_limit', 0.03).value)
        self.gripper_motion_sec = float(self.declare_parameter('gripper_motion_sec', 2.5).value)
        self.gripper_settle_sec = float(self.declare_parameter('gripper_settle_sec', 0.6).value)
        self.staged_lift_offset_z = float(self.declare_parameter('staged_lift_offset_z', 0.03).value)
        self.regrip_after_staged_lift = bool(self.declare_parameter('regrip_after_staged_lift', True).value)
        self.reclamp_before_transport_moves = bool(
            self.declare_parameter('reclamp_before_transport_moves', True).value
        )
        self.transport_time_scale = float(self.declare_parameter('transport_time_scale', 3.5).value)
        self.post_task_wrist_rotate_deg = 90.0
        
        # 0.0 means wait indefinitely for action results.
        self.action_timeout_sec = float(self.declare_parameter('action_timeout_sec', 0.0).value)
        self.fast_exit_on_success = bool(self.declare_parameter('fast_exit_on_success', True).value)
        self.fast_exit_on_failure = bool(self.declare_parameter('fast_exit_on_failure', True).value)

        # Middle cube coordinates expressed directly in pose_frame.
        self.middle_cube_x = float(self.declare_parameter('middle_cube_x', 0.7).value)
        self.middle_cube_y = float(self.declare_parameter('middle_cube_y', 0.0).value)
        self.middle_cube_z = float(self.declare_parameter('middle_cube_z', 0.28).value)

        # Left and right cube Y coordinates for slot-based picking
        # From camera view: left=blue (y=0.25), middle=green (y=0.0), right=red (y=-0.25)
        self.left_cube_y = float(self.declare_parameter('left_cube_y', 0.25).value)
        self.right_cube_y = float(self.declare_parameter('right_cube_y', -0.25).value)

        # Big cube placement location (drop small cube on top)
        # Big cube is at (0.0, 6.0), approach from -0.5 on X axis
        self.big_cube_approach_x = float(self.declare_parameter('big_cube_approach_x', -0.5).value)
        self.big_cube_y = float(self.declare_parameter('big_cube_y', 6.0).value)
        self.big_cube_place_z = float(self.declare_parameter('big_cube_place_z', 0.45).value)
        self.nav_max_lin_speed = float(self.declare_parameter('nav_max_lin_speed', 0.20).value)
        self.nav_min_lin_speed = float(self.declare_parameter('nav_min_lin_speed', 0.04).value)
        self.nav_max_ang_speed = float(self.declare_parameter('nav_max_ang_speed', 0.30).value)
        self.nav_min_ang_speed = float(self.declare_parameter('nav_min_ang_speed', 0.06).value)
        self.nav_heading_kp = float(self.declare_parameter('nav_heading_kp', 1.3).value)
        self.nav_lin_tolerance = float(self.declare_parameter('nav_lin_tolerance', 0.08).value)
        self.nav_ang_tolerance = float(self.declare_parameter('nav_ang_tolerance', 0.05).value)
        self.nav_turn_only_threshold = float(self.declare_parameter('nav_turn_only_threshold', 0.18).value)
        self.nav_timeout_sec = float(self.declare_parameter('nav_timeout_sec', 80.0).value)

        # Use MoveIt planning frame directly to avoid missing world-frame TF.
        self.pose_frame = str(self.declare_parameter('pose_frame', 'base_footprint').value)

        # Vertical offsets from cube center for approach/grasp/lift.
        self.clearance_offset_z = float(self.declare_parameter('clearance_offset_z', 0.22).value)
        self.approach_offset_z = float(self.declare_parameter('approach_offset_z', 0.14).value)
        self.final_approach_offset_z = float(self.declare_parameter('final_approach_offset_z', 0.06).value)
        self.grasp_offset_z = float(self.declare_parameter('grasp_offset_z', 0.015).value)
        self.lift_offset_z = float(self.declare_parameter('lift_offset_z', 0.20).value)
        self.pre_grasp_settle_sec = float(self.declare_parameter('pre_grasp_settle_sec', 0.8).value)

        # End-effector orientation in pose_frame (radians).
        # Default pitch=-pi flips the tool to a top-down style approach.
        self.ee_roll = float(self.declare_parameter('ee_roll', 0.0).value)
        self.ee_pitch = float(self.declare_parameter('ee_pitch', -math.pi).value)
        self.ee_yaw = float(self.declare_parameter('ee_yaw', 0.0).value)
        self.pose_link = str(self.declare_parameter('pose_link', 'ur_tool0').value)

        self._wait_for_joint_states()

    def _subscribe_joint_state_topic(self, topic_name):
        topic_name = str(topic_name).strip()
        if not topic_name:
            return
        if not topic_name.startswith('/'):
            topic_name = f'/{topic_name}'
        if topic_name in self._joint_state_topics:
            return

        # Subscribe with both best-effort and reliable QoS to match varied publishers.
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20,
        )
        transient_local_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._joint_state_subscriptions.append(
            self.create_subscription(JointState, topic_name, self._joint_state_callback, qos_profile_sensor_data)
        )
        self._joint_state_subscriptions.append(
            self.create_subscription(JointState, topic_name, self._joint_state_callback, reliable_qos)
        )
        self._joint_state_subscriptions.append(
            self.create_subscription(JointState, topic_name, self._joint_state_callback, transient_local_qos)
        )
        self._joint_state_topics.add(topic_name)
        self.get_logger().info(f'Subscribed to joint states topic: {topic_name}')

    def _discover_and_subscribe_joint_state_topics(self):
        try:
            topic_map = self.get_topic_names_and_types()
        except Exception as e:
            self.get_logger().warn(f'Could not query available topics: {e}')
            return

        for topic_name, type_names in topic_map:
            if 'sensor_msgs/msg/JointState' not in type_names:
                continue
            if topic_name == '/joint_states' or topic_name.endswith('/joint_states'):
                self._subscribe_joint_state_topic(topic_name)

    def _joint_state_callback(self, msg):
        if msg.name:
            self._joint_state_received = True
            for name, position in zip(msg.name, msg.position):
                self._latest_joint_positions[name] = position

    def _wait_for_joint_states(self, timeout_sec=5.0):
        start = time.time()
        next_discovery_time = start
        while time.time() - start < timeout_sec and rclpy.ok() and not self._joint_state_received:
            now = time.time()
            if now >= next_discovery_time:
                self._discover_and_subscribe_joint_state_topics()
                next_discovery_time = now + 1.0
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._joint_state_received:
            self.get_logger().info('Received initial joint states')
        else:
            watched_topics = ', '.join(sorted(self._joint_state_topics)) if self._joint_state_topics else '(none)'
            self.get_logger().warn(
                f'No joint states received before planning start. Watched topics: {watched_topics}'
            )

    def _small_color_cb(self, msg):
        self.small_cube_color = msg.data
        self.get_logger().info(f'Received small cube color: {self.small_cube_color}')

    def _big_target_cb(self, msg):
        self.big_cube_target = msg.data
        self.get_logger().info(f'Received target big cube: {self.big_cube_target}')

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.current_x = float(p.x)
        self.current_y = float(p.y)

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _build_moveit_config(self):
        omni_moveit2_dir = get_package_share_directory('omni_moveit2')
        omni_description_dir = get_package_share_directory('omni_description')

        robot_description_xml = subprocess.check_output(
            ['xacro', os.path.join(omni_description_dir, 'urdf', 'omni_robot.urdf.xacro')],
            text=True,
        )

        def load_yaml(path):
            with open(path, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)

        config_blocks = [
            {'robot_description': robot_description_xml},
            {
                'robot_description_semantic': open(
                    os.path.join(omni_moveit2_dir, 'config', 'omni_robot.srdf'),
                    'r',
                    encoding='utf-8',
                ).read()
            },
            {
                'robot_description_kinematics': load_yaml(
                    os.path.join(omni_moveit2_dir, 'config', 'kinematics.yaml')
                )
            },
            {
                'planning_pipelines': {
                    'pipeline_names': ['ompl'],
                    'default_planning_pipeline': 'ompl',
                }
            },
            {
                'ompl': load_yaml(
                    os.path.join(omni_moveit2_dir, 'config', 'moveit_planning.yaml')
                )
            },
            load_yaml(os.path.join(omni_moveit2_dir, 'config', 'moveit_controllers.yaml')),
            {
                'trajectory_execution': {
                    'allowed_execution_duration_scaling': 1.5,
                    'allowed_goal_duration_margin': 0.75,
                    'moveit_manage_controllers': False,
                }
            },
            {
                'plan_request_params': {
                    'planner_id': 'RRTConnectkConfigDefault',
                    'planning_pipeline': 'ompl',
                    'planning_time': 20.0,
                    'planning_attempts': 3,
                    'max_velocity_scaling_factor': 1.0,
                    'max_acceleration_scaling_factor': 1.0,
                }
            },
            {
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
                'publish_robot_description': True,
                'publish_robot_description_semantic': True,
            },
        ]

        config = {}
        for block in config_blocks:
            config.update(block)
        return config

    def wait_for_servers(self, timeout_sec: float = 15.0) -> bool:
        self.get_logger().info('Waiting for UR3 and gripper action servers...')
        if not self.ur3_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('UR3 controller action server not available.')
            return False
        if not self.gripper_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('Gripper controller action server not available.')
            return False
        self.get_logger().info('Controllers are ready.')
        return True

    def _send_trajectory(self, client: ActionClient, joint_names, positions, duration_sec: float, wait_for_result: bool = True) -> bool:
        self.get_logger().info(
            f'Sending trajectory: joints={joint_names}, positions={[round(p, 4) for p in positions]}, duration={duration_sec:.2f}s'
        )
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        goal.trajectory.points = [point]

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        if not send_future.done():
            self.get_logger().error('Timed out waiting for action server to accept goal.')
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Trajectory goal was rejected.')
            return False

        if not wait_for_result:
            self.get_logger().info('Goal sent (non-blocking mode).')
            return True

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.action_timeout_sec)
        if not result_future.done():
            self.get_logger().error(f'Timed out waiting for trajectory result ({self.action_timeout_sec:.1f}s).')
            return False

        result_wrap = result_future.result()
        if result_wrap is None:
            self.get_logger().error('No result received from trajectory action.')
            return False

        if result_wrap.status != 4:
            self.get_logger().error(f'Trajectory execution failed with status {result_wrap.status}.')
            return False

        return True

    def _duration_to_seconds(self, duration_msg):
        return float(duration_msg.sec) + float(duration_msg.nanosec) * 1e-9

    def _set_duration_seconds(self, duration_msg, seconds):
        sec = int(seconds)
        nanosec = int((seconds - sec) * 1e9)
        duration_msg.sec = sec
        duration_msg.nanosec = nanosec

    def _scale_trajectory_timing(self, trajectory, scale):
        if scale <= 1.0 or not trajectory.points:
            return

        for pt in trajectory.points:
            t = self._duration_to_seconds(pt.time_from_start)
            self._set_duration_seconds(pt.time_from_start, t * scale)

        self._ensure_strictly_increasing_timestamps(trajectory)

    def _ensure_strictly_increasing_timestamps(self, trajectory):
        if not trajectory.points:
            return

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

        if len(trajectory.points) > 1 and self._duration_to_seconds(trajectory.points[1].time_from_start) <= 0.0:
            self._set_duration_seconds(trajectory.points[1].time_from_start, dt)

    def _trajectory_duration_seconds(self, trajectory):
        if not trajectory.points:
            return 0.0
        return self._duration_to_seconds(trajectory.points[-1].time_from_start)

    def _result_wait_timeout_for_trajectory(self, trajectory):
        expected = self._trajectory_duration_seconds(trajectory)
        # Non-positive timeout disables forced wall-clock timeout.
        if self.action_timeout_sec <= 0.0:
            return 0.0
        # Keep a minimum timeout while also allowing long trajectories to finish.
        return max(self.action_timeout_sec, expected + 20.0)

    def _wait_for_joint_target(self, joint_names, target_positions, timeout_sec, tolerance=0.03, settle_sec=0.4):
        start = time.time()
        reached_since = None

        while rclpy.ok() and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

            if not all(name in self._latest_joint_positions for name in joint_names):
                reached_since = None
                continue

            errors = [
                abs(self._latest_joint_positions[name] - target)
                for name, target in zip(joint_names, target_positions)
            ]
            max_err = max(errors) if errors else float('inf')

            if max_err <= tolerance:
                if reached_since is None:
                    reached_since = time.time()
                elif (time.time() - reached_since) >= settle_sec:
                    self.get_logger().info(
                        f'Joint-state convergence reached (max_err={max_err:.4f} rad)'
                    )
                    return True
            else:
                reached_since = None

        return False

    def _execute_ur3_trajectory(self, joint_traj, description):
        if self._holding_object and self.transport_time_scale > 1.0:
            self.get_logger().info(
                f'Holding object: scaling trajectory timing by x{self.transport_time_scale:.2f} for {description}'
            )
            self._scale_trajectory_timing(joint_traj, self.transport_time_scale)

        expected = self._trajectory_duration_seconds(joint_traj)
        configured_wait_timeout = self._result_wait_timeout_for_trajectory(joint_traj)
        log_wait = 'infinite' if configured_wait_timeout <= 0.0 else f'{configured_wait_timeout:.2f}s'
        self.get_logger().info(
            f'Executing {description}: waypoints={len(joint_traj.points)}, '
            f'expected_duration={expected:.2f}s, wait_timeout={log_wait}'
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj
        send_future = self.ur3_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        if not send_future.done():
            self.get_logger().error('Timed out waiting for UR3 goal acceptance.')
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'Execution rejected: {description}')
            return False

        # Some controller stacks accept goals but never publish a terminal result.
        # Wait bounded time for action result, then fall back to joint-state convergence.
        result_wait = configured_wait_timeout
        if result_wait <= 0.0:
            result_wait = max(expected + 15.0, 20.0)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=result_wait)
        if result_future.done():
            result_wrap = result_future.result()
            if result_wrap is None or result_wrap.status != 4:
                status = 'none' if result_wrap is None else result_wrap.status
                self.get_logger().error(f'Execution failed for {description}, status={status}')
                return False
            return True

        self.get_logger().warn(
            f'No UR3 action result after {result_wait:.1f}s in {description}; '
            'checking joint-state convergence instead.'
        )

        if not joint_traj.points:
            self.get_logger().error('UR3 trajectory has no points; cannot validate execution.')
            return False

        target_positions = list(joint_traj.points[-1].positions)
        monitor_timeout = max(expected + 30.0, 30.0)
        converged = self._wait_for_joint_target(
            joint_traj.joint_names,
            target_positions,
            timeout_sec=monitor_timeout,
        )
        if converged:
            self.get_logger().info(
                f'Accepted {description} as completed via joint-state convergence.'
            )
            return True

        self.get_logger().error(
            f'UR3 did not reach target for {description} within {monitor_timeout:.1f}s; canceling goal.'
        )
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
        return False

    def _plan_to_joint_goal(self, group_name, joint_positions):
        planning_component = self.moveit.get_planning_component(group_name)
        robot_state = RobotState(self.moveit.get_robot_model())
        robot_state.set_joint_group_positions(group_name, joint_positions)
        planning_component.set_start_state_to_current_state()
        planning_component.set_goal_state(robot_state=robot_state)
        return planning_component.plan()

    def _plan_to_pose_goal(self, x, y, z):
        try:
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.pose_frame
            target_pose.pose.position.x = float(x)
            target_pose.pose.position.y = float(y)
            target_pose.pose.position.z = float(z)

            qx, qy, qz, qw = self._quat_from_rpy(self.ee_roll, self.ee_pitch, self.ee_yaw)
            target_pose.pose.orientation.x = qx
            target_pose.pose.orientation.y = qy
            target_pose.pose.orientation.z = qz
            target_pose.pose.orientation.w = qw

            planning_component = self.moveit.get_planning_component('ur3_arm')
            planning_component.set_start_state_to_current_state()
            planning_component.set_goal_state(
                pose_stamped_msg=target_pose,
                pose_link=self.pose_link,
            )
            return planning_component.plan()
        except Exception as e:
            self.get_logger().error(f'Pose planning API error: {e}')
            return None

    def _quat_from_rpy(self, roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def _execute_arm_plan_result(self, plan_result, description):
        if not plan_result:
            self.get_logger().error(f'Planning failed: {description}')
            return False

        robot_traj_msg = plan_result.trajectory.get_robot_trajectory_msg()
        joint_traj = robot_traj_msg.joint_trajectory
        self._ensure_strictly_increasing_timestamps(joint_traj)
        return self._execute_ur3_trajectory(joint_traj, description)

    def _plan_pose_and_execute(self, x, y, z, description):
        self.get_logger().info(f'Planning pose {description}: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        plan_result = self._plan_to_pose_goal(x, y, z)
        return self._execute_arm_plan_result(plan_result, description)

    def _execute_pose_with_joint_fallback(self, x, y, z, description, fallback_joint_pose):
        if self._plan_pose_and_execute(x, y, z, description):
            return True

        self.get_logger().warn(
            f'Pose planning failed for {description}. Falling back to joint waypoint execution.'
        )
        return self._plan_and_execute('ur3_arm', fallback_joint_pose, f'{description} (joint fallback)')

    def _plan_and_execute(self, group_name, joint_positions, description):
        if group_name == 'gripper':
            cmd_left = min(self.gripper_max_limit, max(self.gripper_min_limit, float(joint_positions[0])))
            cmd_right = min(self.gripper_max_limit, max(self.gripper_min_limit, float(joint_positions[1])))
            if cmd_left != joint_positions[0] or cmd_right != joint_positions[1]:
                self.get_logger().warn(
                    f'Clamped gripper command from {joint_positions} to {[cmd_left, cmd_right]}'
                )

            self.get_logger().info(f'Executing direct gripper command: {description}')
            ok = self._send_trajectory(
                self.gripper_client,
                ['gripper_left_finger_joint', 'gripper_right_finger_joint'],
                [cmd_left, cmd_right],
                self.gripper_motion_sec,
                wait_for_result=False,
            )
            # Gripper controller can stall without returning action result in Gazebo.
            # Allow motion time and continue sequence.
            if ok:
                time.sleep(self.gripper_motion_sec + self.gripper_settle_sec)
            return ok

        self.get_logger().info(f'Planning: {description}')
        plan_result = self._plan_to_joint_goal(group_name, joint_positions)
        if not plan_result:
            self.get_logger().error(f'Planning failed: {description}')
            return False

        robot_traj_msg = plan_result.trajectory.get_robot_trajectory_msg()
        joint_traj = robot_traj_msg.joint_trajectory
        self._ensure_strictly_increasing_timestamps(joint_traj)
        return self._execute_ur3_trajectory(joint_traj, description)

    def _secure_grasp(self):
        # First close to nominal grasp width.
        if not self._plan_and_execute(
            'gripper',
            [self.gripper_close_pos, self.gripper_close_pos],
            'Close gripper',
        ):
            return False

        if not self.enable_gripper_squeeze:
            return True

        # Then apply a small extra squeeze to improve grip on slippery contacts.
        squeeze_pos = max(self.gripper_min_limit, self.gripper_close_pos + self.gripper_squeeze_delta)
        if squeeze_pos >= self.gripper_close_pos:
            return True

        return self._plan_and_execute(
            'gripper',
            [squeeze_pos, squeeze_pos],
            'Squeeze gripper',
        )

    def _current_arm_joint_positions(self):
        arm_joint_names = [
            'ur_shoulder_pan_joint',
            'ur_shoulder_lift_joint',
            'ur_elbow_joint',
            'ur_wrist_1_joint',
            'ur_wrist_2_joint',
            'ur_wrist_3_joint',
        ]

        positions = []
        for name in arm_joint_names:
            if name not in self._latest_joint_positions:
                return None
            positions.append(self._latest_joint_positions[name])
        return positions

    def _rotate_wrist_after_lift(self):
        current = self._current_arm_joint_positions()
        if current is None:
            self.get_logger().warn('Cannot rotate wrist: current arm joint states are incomplete')
            return False

        delta = math.radians(self.post_task_wrist_rotate_deg)
        current[5] = current[5] + delta
        self.get_logger().info(
            f'Rotating end-effector wrist by {self.post_task_wrist_rotate_deg:.1f} deg after task motion'
        )
        return self._plan_and_execute('ur3_arm', current, 'Post-lift wrist rotate')

    def _pre_transport_reclamp(self):
        if not self._holding_object:
            return True
        if not self.reclamp_before_transport_moves:
            return True

        self.get_logger().info('Re-clamping gripper before transport move')
        return self._secure_grasp()

    def move_arm(self, positions, duration_sec: float = 3.0) -> bool:
        return self._send_trajectory(
            self.ur3_client,
            [
                'ur_shoulder_pan_joint',
                'ur_shoulder_lift_joint',
                'ur_elbow_joint',
                'ur_wrist_1_joint',
                'ur_wrist_2_joint',
                'ur_wrist_3_joint',
            ],
            positions,
            duration_sec,
        )

    def move_gripper(self, opening: float, duration_sec: float = 1.5) -> bool:
        return self._send_trajectory(
            self.gripper_client,
            ['gripper_left_finger_joint', 'gripper_right_finger_joint'],
            [opening, opening],
            duration_sec,
        )

    def _wait_for_small_color(self, timeout_sec=10.0):
        start = time.time()
        while time.time() - start < timeout_sec and self.small_cube_color is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.small_cube_color is not None:
            self.get_logger().info(f'Small cube color determined: {self.small_cube_color}')
            return True
        self.get_logger().error('Timeout waiting for small cube color information')
        return False

    def _wait_for_big_target(self, timeout_sec=20.0):
        start = time.time()
        while time.time() - start < timeout_sec and self.big_cube_target is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.big_cube_target is not None:
            self.get_logger().info(f'Target big cube determined: {self.big_cube_target}')
            return True
        self.get_logger().error('Timeout waiting for target big-cube information')
        return False

    def _get_pick_y_for_slot(self):
        # New setup spawns exactly one small cube at table center.
        return self.middle_cube_y

    def _get_big_cube_goal(self):
        # Big cube world positions: left, behind, right.
        if self.big_cube_target == 'left':
            return -0.4, 1.0
        if self.big_cube_target == 'behind':
            return -1.9, 0.0
        if self.big_cube_target == 'right':
            return -0.4, -1.0
        # Fallback to existing parameters.
        return self.big_cube_approach_x, self.big_cube_y

    def _adjust_pick_position_for_drift(self):
        """Adjust pick coordinates based on actual robot position (odometry drift compensation)"""
        if self.current_x is None or self.current_y is None:
            self.get_logger().warn('No odometry data available, picking at default position')
            return self.middle_cube_x, self._get_pick_y_for_slot()
        
        # Calculate offset from assumed origin (0, 0)
        drift_x = self.current_x
        drift_y = self.current_y
        
        if abs(drift_x) > 0.01 or abs(drift_y) > 0.01:
            self.get_logger().info(
                f'Odometry drift detected: x_drift={drift_x:.3f}, y_drift={drift_y:.3f}'
            )
        
        # Adjust pick coordinates by drift amount
        adjusted_x = self.middle_cube_x - drift_x
        adjusted_y = self._get_pick_y_for_slot() - drift_y
        
        self.get_logger().info(
            f'Adjusted pick position: x={adjusted_x:.3f} (was {self.middle_cube_x:.3f}), y={adjusted_y:.3f}'
        )
        
        return adjusted_x, adjusted_y

    def _navigate_to_big_cube(self):
        """Navigate base to big cube with turn-then-drive control for better accuracy."""
        self.get_logger().info(f'Navigating to big cube approach at ({self.big_cube_approach_x:.2f}, {self.big_cube_y:.2f})')

        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < self.nav_timeout_sec:
            if self.current_x is None or self.current_y is None or self.current_yaw is None:
                rclpy.spin_once(self, timeout_sec=0.05)
                continue

            dx = self.big_cube_approach_x - self.current_x
            dy = self.big_cube_y - self.current_y
            dist = math.hypot(dx, dy)

            if dist < self.nav_lin_tolerance:
                self.cmd_pub.publish(Twist())
                rclpy.spin_once(self, timeout_sec=0.05)
                self.cmd_pub.publish(Twist())
                self.get_logger().info('Reached big cube approach location')
                return True

            target_heading = math.atan2(dy, dx)
            heading_err = target_heading - self.current_yaw
            while heading_err > math.pi:
                heading_err -= 2.0 * math.pi
            while heading_err < -math.pi:
                heading_err += 2.0 * math.pi

            abs_err = abs(heading_err)
            if abs_err > self.nav_turn_only_threshold:
                lin = 0.0
                ang = self.nav_heading_kp * heading_err
                if abs(ang) < self.nav_min_ang_speed:
                    ang = math.copysign(self.nav_min_ang_speed, heading_err)
                ang = max(-self.nav_max_ang_speed, min(self.nav_max_ang_speed, ang))
            else:
                # Slow down near target and when heading error is not yet tiny.
                lin = min(self.nav_max_lin_speed, 0.55 * dist)
                if lin < self.nav_min_lin_speed:
                    lin = self.nav_min_lin_speed
                if abs_err > self.nav_ang_tolerance:
                    lin *= 0.35

                ang = self.nav_heading_kp * heading_err
                if abs(ang) < self.nav_min_ang_speed and abs_err > self.nav_ang_tolerance:
                    ang = math.copysign(self.nav_min_ang_speed, heading_err)
                ang = max(-self.nav_max_ang_speed, min(self.nav_max_ang_speed, ang))

            cmd = Twist()
            cmd.linear.x = lin
            cmd.angular.z = ang
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.cmd_pub.publish(Twist())
        self.get_logger().error('Navigation timeout')
        return False

    def _place_on_big_cube(self, target_x, target_y, target_z):
        """Place the held cube on top of the big cube"""
        self.get_logger().info(f'Placing cube on big cube at ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})')
        
        clearance_z = target_z + self.clearance_offset_z
        approach_z = target_z + self.approach_offset_z
        place_z = target_z + 0.01
        
        if not self._execute_pose_with_joint_fallback(
            target_x, target_y, clearance_z, 'Place clearance', self.lift_pose
        ):
            return False
        time.sleep(0.3)
        
        if not self._execute_pose_with_joint_fallback(
            target_x, target_y, approach_z, 'Place approach', self.lift_pose
        ):
            return False
        time.sleep(0.3)
        
        if not self._execute_pose_with_joint_fallback(
            target_x, target_y, place_z, 'Place descend', self.grasp_pose
        ):
            return False
        time.sleep(0.3)
        
        if not self._plan_and_execute('gripper', [self.gripper_open_pos, self.gripper_open_pos], 'Open gripper'):
            return False
        self._holding_object = False
        time.sleep(0.5)
        
        if not self._execute_pose_with_joint_fallback(
            target_x, target_y, clearance_z, 'Place retreat', self.lift_pose
        ):
            return False
        
        return self._plan_and_execute('ur3_arm', self.home_pose, 'Return home after place')

    def run(self):
        if not MOVEIT_AVAILABLE:
            return False

        self._holding_object = False

        if not self._joint_state_received:
            self._wait_for_joint_states(timeout_sec=10.0)
        if not self._joint_state_received:
            self.get_logger().error('No joint states received. Refusing to execute pick sequence.')
            return False

        # Wait for small-cube color information from detect_small_cubes node.
        if not self._wait_for_small_color(timeout_sec=30.0):
            return False

        if not self.wait_for_servers():
            return False

        self.get_logger().info(
            f'Starting puzzle pick-place sequence for small color: {self.small_cube_color}'
        )

        if not self._plan_and_execute('gripper', [self.gripper_open_pos, self.gripper_open_pos], 'Open gripper'):
            return False
        time.sleep(0.5)

        # Get drift-adjusted pick position based on actual odometry
        target_x, target_y = self._adjust_pick_position_for_drift()
        target_z = self.middle_cube_z

        clearance_z = target_z + self.clearance_offset_z
        approach_z = target_z + self.approach_offset_z
        final_approach_z = target_z + self.final_approach_offset_z
        grasp_z = target_z + self.grasp_offset_z
        lift_z = target_z + self.lift_offset_z

        self.get_logger().info(
            f'Picking single table cube: target=({target_x:.3f}, {target_y:.3f}, {target_z:.3f})'
        )

        if not self._execute_pose_with_joint_fallback(
            target_x,
            target_y,
            clearance_z,
            'Move to high clearance pose',
            self.pre_grasp_pose,
        ):
            return False
        time.sleep(0.5)

        if not self._execute_pose_with_joint_fallback(
            target_x,
            target_y,
            approach_z,
            'Move to pre-grasp pose',
            self.pre_grasp_pose,
        ):
            return False
        time.sleep(self.pre_grasp_settle_sec)

        if not self._execute_pose_with_joint_fallback(
            target_x,
            target_y,
            final_approach_z,
            'Move to final approach pose',
            self.pre_grasp_pose,
        ):
            return False
        time.sleep(self.pre_grasp_settle_sec)

        if not self._execute_pose_with_joint_fallback(
            target_x,
            target_y,
            grasp_z,
            'Descend to grasp pose',
            self.grasp_pose,
        ):
            return False
        time.sleep(self.pre_grasp_settle_sec)

        if not self._secure_grasp():
            return False
        self._holding_object = True
        time.sleep(0.5)

        staged_lift_z = target_z + self.staged_lift_offset_z
        if not self._pre_transport_reclamp():
            return False
        if not self._execute_pose_with_joint_fallback(
            target_x,
            target_y,
            staged_lift_z,
            'Lift slightly to seat grasp',
            self.lift_pose,
        ):
            return False
        time.sleep(self.pre_grasp_settle_sec)

        if self.regrip_after_staged_lift:
            if not self._secure_grasp():
                return False
            time.sleep(0.4)

        if not self._pre_transport_reclamp():
            return False
        if not self._execute_pose_with_joint_fallback(
            target_x,
            target_y,
            lift_z,
            'Lift cube',
            self.lift_pose,
        ):
            return False
        time.sleep(0.5)

        if not self._plan_and_execute('ur3_arm', self.home_pose, 'Return home after pick'):
            return False
        time.sleep(0.3)

        # Publish picked cube color so rotate_scan_color can find matching big cube.
        color_msg = String()
        color_msg.data = self.small_cube_color
        self.picked_color_pub.publish(color_msg)
        start_msg = Bool()
        start_msg.data = True
        self.scan_start_pub.publish(start_msg)
        self.get_logger().info(
            f'Published picked cube color: {self.small_cube_color} to /color_match/picked_cube_color and triggered /color_match/start_scan'
        )

        # Wait for rotate_scan_color node to publish target big cube.
        if not self._wait_for_big_target(timeout_sec=180.0):
            return False

        self.big_cube_approach_x, self.big_cube_y = self._get_big_cube_goal()
        self.get_logger().info(
            f'Navigating to matched big cube target={self.big_cube_target} at '
            f'({self.big_cube_approach_x:.2f}, {self.big_cube_y:.2f})'
        )

        # Navigate to big cube location
        if not self._navigate_to_big_cube():
            return False
        time.sleep(0.5)

        # Place cube on big cube
        place_x = self.big_cube_approach_x
        place_y = self.big_cube_y
        place_z = self.big_cube_place_z
        if not self._place_on_big_cube(place_x, place_y, place_z):
            return False

        self.get_logger().info('Puzzle task completed successfully!')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PickMiddleCube()
    success = False
    try:
        success = node.run()
    finally:
        if success and getattr(node, 'fast_exit_on_success', False):
            # Workaround: MoveItPy teardown can segfault in some Jazzy/Gazebo setups.
            os._exit(0)
        if (not success) and getattr(node, 'fast_exit_on_failure', False):
            # Same teardown workaround for failure path.
            os._exit(1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import math
import os
import subprocess
import time
import yaml

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False


ARM_JOINTS = [
    'ur_shoulder_pan_joint',
    'ur_shoulder_lift_joint',
    'ur_elbow_joint',
    'ur_wrist_1_joint',
    'ur_wrist_2_joint',
    'ur_wrist_3_joint',
]

GRIPPER_JOINTS = [
    'gripper_left_finger_joint',
    'gripper_right_finger_joint',
]


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PuzzlePickPlaceNode(Node):
    def __init__(self):
        super().__init__('puzzle_pick_place')

        if not MOVEIT_AVAILABLE:
            self.get_logger().error('MoveIt2 Python API is not available.')
            raise RuntimeError('moveit_py unavailable')

        self.bridge = CvBridge()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self._image_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 20)

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

        self.moveit = MoveItPy(
            node_name='puzzle_pick_place_moveit',
            config_dict=self._build_moveit_config(),
        )

        self.angular_speed = float(self.declare_parameter('angular_speed', 0.25).value)
        self.linear_speed_max = float(self.declare_parameter('linear_speed_max', 0.35).value)
        self.target_scan_deg = float(self.declare_parameter('target_scan_deg', 90.0).value)
        self.yaw_tolerance = float(self.declare_parameter('yaw_tolerance', 0.04).value)
        self.timer_period = float(self.declare_parameter('timer_period', 0.05).value)

        self.pose_frame = str(self.declare_parameter('pose_frame', 'base_footprint').value)
        self.pose_link = str(self.declare_parameter('pose_link', 'ur_tool0').value)
        self.ee_roll = float(self.declare_parameter('ee_roll', 0.0).value)
        self.ee_pitch = float(self.declare_parameter('ee_pitch', -math.pi).value)
        self.ee_yaw = float(self.declare_parameter('ee_yaw', 0.0).value)

        self.home_pose = [0.0, -1.57, 1.57, -1.57, 0.0, 0.0]
        self.gripper_open = float(self.declare_parameter('gripper_open_pos', 0.015).value)
        self.gripper_close = float(self.declare_parameter('gripper_close_pos', -0.075).value)

        self.small_x = float(self.declare_parameter('small_cube_x', 0.70).value)
        self.small_y_left = float(self.declare_parameter('small_cube_y_left', 0.25).value)
        self.small_y_middle = float(self.declare_parameter('small_cube_y_middle', 0.0).value)
        self.small_y_right = float(self.declare_parameter('small_cube_y_right', -0.25).value)
        self.small_z = float(self.declare_parameter('small_cube_z', 0.215).value)

        self.big_nav_x = float(self.declare_parameter('big_nav_x', -0.7).value)
        self.big_nav_y = float(self.declare_parameter('big_nav_y', 6.0).value)
        self.big_nav_yaw = float(self.declare_parameter('big_nav_yaw', 0.0).value)

        self.place_x = float(self.declare_parameter('place_x', 0.70).value)
        self.place_y = float(self.declare_parameter('place_y', 0.0).value)
        self.place_z = float(self.declare_parameter('place_z', 0.235).value)

        self.clearance_offset_z = float(self.declare_parameter('clearance_offset_z', 0.22).value)
        self.approach_offset_z = float(self.declare_parameter('approach_offset_z', 0.12).value)
        self.grasp_offset_z = float(self.declare_parameter('grasp_offset_z', 0.02).value)
        self.lift_offset_z = float(self.declare_parameter('lift_offset_z', 0.22).value)

        self.min_area_small = int(self.declare_parameter('min_area_small', 150).value)
        self.min_area_big = int(self.declare_parameter('min_area_big', 500).value)

        self.fast_exit_on_success = bool(self.declare_parameter('fast_exit_on_success', True).value)
        self.fast_exit_on_failure = bool(self.declare_parameter('fast_exit_on_failure', True).value)
        self.task_success = False

        self.phase = 'WAIT_ODOM'
        self.done = False

        self.start_yaw = None
        self.current_yaw = None
        self.current_x = None
        self.current_y = None

        self.big_scores = {'red': 0, 'green': 0, 'blue': 0}
        self.big_color = None
        self.small_mapping = None
        self.target_slot = None

        self.goal_handle_future = None
        self.result_future = None

        self.get_logger().info('Puzzle node started: scan big color, match small cube, pick, navigate, place.')

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
        ]

        config = {}
        for block in config_blocks:
            config.update(block)
        return config

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.current_x = float(p.x)
        self.current_y = float(p.y)

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _build_color_masks(self, hsv):
        red1 = cv2.inRange(hsv, (0, 80, 50), (10, 255, 255))
        red2 = cv2.inRange(hsv, (170, 80, 50), (180, 255, 255))
        red = cv2.bitwise_or(red1, red2)
        green = cv2.inRange(hsv, (40, 80, 50), (85, 255, 255))
        blue = cv2.inRange(hsv, (95, 80, 50), (130, 255, 255))
        return {'red': red, 'green': green, 'blue': blue}

    def _image_cb(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')
            return

        if self.phase == 'ROTATE_LEFT_SCAN':
            h, w = image.shape[:2]
            roi = image[int(0.2 * h):int(0.85 * h), int(0.2 * w):int(0.8 * w)]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            masks = self._build_color_masks(hsv)

            for name, mask in masks.items():
                area = int(cv2.countNonZero(mask))
                if area > self.min_area_big:
                    self.big_scores[name] = max(self.big_scores[name], area)

        if self.phase == 'DETECT_SMALL' and self.small_mapping is None:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            masks = self._build_color_masks(hsv)

            candidates = []
            for name, mask in masks.items():
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area < self.min_area_small:
                        continue
                    m = cv2.moments(cnt)
                    if m['m00'] == 0.0:
                        continue
                    cx = float(m['m10'] / m['m00'])
                    candidates.append((cx, area, name))

            if len(candidates) >= 3:
                candidates.sort(key=lambda x: x[1], reverse=True)
                top = candidates[:3]
                top.sort(key=lambda x: x[0])
                labels = ['left', 'middle', 'right']
                self.small_mapping = {labels[i]: top[i][2] for i in range(3)}
                self.get_logger().info(
                    f"Small cubes: left={self.small_mapping['left']}, middle={self.small_mapping['middle']}, right={self.small_mapping['right']}"
                )

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_twist(self, linear_x=0.0, angular_z=0.0):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def _send_joint_goal(self, client, joint_names, positions, duration_sec=3.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = joint_names

        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start.sec = int(duration_sec)
        pt.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        goal.trajectory.points = [pt]

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=6.0)
        if not send_future.done():
            self.get_logger().error('Action goal acceptance timeout.')
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Action goal rejected.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=max(duration_sec + 15.0, 20.0))
        if not result_future.done():
            self.get_logger().error('Action result timeout.')
            return False

        result_wrap = result_future.result()
        if result_wrap is None or result_wrap.status != 4:
            status = 'none' if result_wrap is None else result_wrap.status
            self.get_logger().error(f'Action execution failed. status={status}')
            return False

        return True

    def _wait_for_servers(self):
        if not self.ur3_client.wait_for_server(timeout_sec=8.0):
            self.get_logger().error('UR3 action server not available.')
            return False
        if not self.gripper_client.wait_for_server(timeout_sec=8.0):
            self.get_logger().error('Gripper action server not available.')
            return False
        return True

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

    def _plan_to_pose_goal(self, x, y, z):
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
        planning_component.set_goal_state(pose_stamped_msg=target_pose, pose_link=self.pose_link)
        return planning_component.plan()

    def _plan_to_joint_goal(self, joint_positions):
        planning_component = self.moveit.get_planning_component('ur3_arm')
        robot_state = RobotState(self.moveit.get_robot_model())
        robot_state.set_joint_group_positions('ur3_arm', list(joint_positions))
        planning_component.set_start_state_to_current_state()
        planning_component.set_goal_state(robot_state=robot_state)
        return planning_component.plan()

    def _execute_plan(self, plan_result, label):
        if not plan_result:
            self.get_logger().error(f'Planning failed: {label}')
            return False

        joint_traj = plan_result.trajectory.get_robot_trajectory_msg().joint_trajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj

        send_future = self.ur3_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=6.0)
        if not send_future.done():
            self.get_logger().error(f'UR3 accept timeout: {label}')
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'UR3 goal rejected: {label}')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        if not result_future.done():
            self.get_logger().error(f'UR3 result timeout: {label}')
            return False

        result_wrap = result_future.result()
        if result_wrap is None or result_wrap.status != 4:
            status = 'none' if result_wrap is None else result_wrap.status
            self.get_logger().error(f'UR3 execution failed ({label}), status={status}')
            return False
        return True

    def _move_arm_home(self):
        plan = self._plan_to_joint_goal(self.home_pose)
        return self._execute_plan(plan, 'arm home')

    def _move_arm_pose(self, x, y, z, label):
        plan = self._plan_to_pose_goal(x, y, z)
        return self._execute_plan(plan, label)

    def _move_gripper(self, opening):
        return self._send_joint_goal(self.gripper_client, GRIPPER_JOINTS, [opening, opening], duration_sec=2.2)

    def _slot_to_pick_y(self, slot_name):
        if slot_name == 'left':
            return self.small_y_left
        if slot_name == 'middle':
            return self.small_y_middle
        return self.small_y_right

    def _pick_small_cube_for_color(self):
        if self.big_color is None or self.small_mapping is None:
            return False

        slot = None
        for s in ['left', 'middle', 'right']:
            if self.small_mapping.get(s) == self.big_color:
                slot = s
                break

        if slot is None:
            self.get_logger().error(f'No small cube matches big cube color={self.big_color}.')
            return False

        self.target_slot = slot
        y = self._slot_to_pick_y(slot)
        x = self.small_x
        z = self.small_z

        self.get_logger().info(f'Picking {self.big_color} cube from slot={slot} at y={y:.3f}')

        if not self._move_gripper(self.gripper_open):
            return False
        time.sleep(0.3)

        if not self._move_arm_pose(x, y, z + self.clearance_offset_z, 'pick clearance'):
            return False
        if not self._move_arm_pose(x, y, z + self.approach_offset_z, 'pick approach'):
            return False
        if not self._move_arm_pose(x, y, z + self.grasp_offset_z, 'pick grasp'):
            return False

        if not self._move_gripper(self.gripper_close):
            return False
        time.sleep(0.4)

        if not self._move_arm_pose(x, y, z + self.lift_offset_z, 'pick lift'):
            return False

        return self._move_arm_home()

    def _goto_xy(self, goal_x, goal_y):
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return False

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        dist = math.hypot(dx, dy)
        if dist < 0.10:
            self._publish_stop()
            return True

        target_heading = math.atan2(dy, dx)
        heading_err = normalize_angle(target_heading - self.current_yaw)

        ang = max(-0.6, min(0.6, 1.6 * heading_err))
        if abs(heading_err) > 0.35:
            lin = 0.0
        else:
            lin = min(self.linear_speed_max, 0.25 * dist)

        self._publish_twist(lin, ang)
        return False

    def _align_yaw(self, target_yaw):
        if self.current_yaw is None:
            return False
        err = normalize_angle(target_yaw - self.current_yaw)
        if abs(err) <= self.yaw_tolerance:
            self._publish_stop()
            return True
        self._publish_twist(0.0, max(-0.4, min(0.4, 1.4 * err)))
        return False

    def _place_on_big_cube(self):
        x = self.place_x
        y = self.place_y
        z = self.place_z

        self.get_logger().info('Placing cube on top of big cube.')

        if not self._move_arm_pose(x, y, z + self.clearance_offset_z, 'place clearance'):
            return False
        if not self._move_arm_pose(x, y, z + self.approach_offset_z, 'place approach'):
            return False
        if not self._move_arm_pose(x, y, z + 0.01, 'place descend'):
            return False

        if not self._move_gripper(self.gripper_open):
            return False
        time.sleep(0.4)

        if not self._move_arm_pose(x, y, z + self.lift_offset_z, 'place retreat'):
            return False

        return self._move_arm_home()

    def _finish(self, success=True):
        self._publish_stop()
        self.task_success = bool(success)
        if success:
            self.get_logger().info('Puzzle task completed successfully.')
        else:
            self.get_logger().error('Puzzle task failed.')
        self.done = True
        if rclpy.ok():
            rclpy.shutdown()

    def _control_loop(self):
        if self.done:
            return

        if self.phase == 'WAIT_ODOM':
            if self.current_yaw is None:
                return
            if not self._wait_for_servers():
                self._finish(False)
                return
            self.start_yaw = self.current_yaw
            self.phase = 'ARM_HOME'
            return

        if self.phase == 'ARM_HOME':
            self.get_logger().info('Moving arm to home before scan...')
            if not self._move_arm_home():
                self._finish(False)
                return
            self.phase = 'ROTATE_LEFT_SCAN'
            return

        if self.phase == 'ROTATE_LEFT_SCAN':
            if self.current_yaw is None or self.start_yaw is None:
                return

            delta = abs(normalize_angle(self.current_yaw - self.start_yaw))
            if delta >= math.radians(self.target_scan_deg):
                self._publish_stop()
                self.big_color = max(self.big_scores, key=self.big_scores.get)
                self.get_logger().info(
                    f'Big cube estimated color={self.big_color}, scores={self.big_scores}'
                )
                self.phase = 'ROTATE_BACK'
                return

            self._publish_twist(0.0, abs(self.angular_speed))
            return

        if self.phase == 'ROTATE_BACK':
            if self._align_yaw(self.start_yaw):
                self.get_logger().info('Returned to start heading. Detecting small cubes once...')
                self.phase = 'DETECT_SMALL'
            return

        if self.phase == 'DETECT_SMALL':
            if self.small_mapping is None:
                return
            self.phase = 'PICK_MATCHING'
            return

        if self.phase == 'PICK_MATCHING':
            if not self._pick_small_cube_for_color():
                self._finish(False)
                return
            self.phase = 'NAV_TO_BIG'
            return

        if self.phase == 'NAV_TO_BIG':
            if self._goto_xy(self.big_nav_x, self.big_nav_y):
                self.get_logger().info('Reached big-cube navigation point. Aligning yaw...')
                self.phase = 'ALIGN_FOR_PLACE'
            return

        if self.phase == 'ALIGN_FOR_PLACE':
            if self._align_yaw(self.big_nav_yaw):
                self.phase = 'PLACE'
            return

        if self.phase == 'PLACE':
            if not self._place_on_big_cube():
                self._finish(False)
                return
            self._finish(True)
            return


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PuzzlePickPlaceNode()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=node.timer_period)
            node._control_loop()
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('Interrupted by user.')
    except Exception as e:
        if node is not None:
            node.get_logger().error(f'Fatal error: {e}')
        if rclpy.ok():
            rclpy.shutdown()
    finally:
        if node is not None and getattr(node, 'done', False):
            if getattr(node, 'task_success', False) and getattr(node, 'fast_exit_on_success', False):
                os._exit(0)
            if (not getattr(node, 'task_success', False)) and getattr(node, 'fast_exit_on_failure', False):
                os._exit(1)

        if node is not None and rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

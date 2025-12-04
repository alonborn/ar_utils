#!/usr/bin/env python3
"""
A script to follow an aruco marker with a robot arm using PyMoveit2.
"""
import math
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from my_robot_interfaces.srv import MoveToPose  # Import the custom service type
from geometry_msgs.msg import Pose 
from geometry_msgs.msg import Point 
import threading
import sys
import tf2_ros
from geometry_msgs.msg import PoseStamped,Quaternion
from typing import Optional
import os
import datetime

from geometry_msgs.msg import Pose
import tf_transformations
sys.path.insert(0, '/home/alon/ros_ws/src/pymoveit2')

from pymoveit2 import MoveIt2
import pymoveit2

import logging
import debugpy
import time
from pymoveit2.moveit2 import init_execute_trajectory_goal

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, WorkspaceParameters
import threading
import cProfile
import pstats
import cProfile
import pstats
import cProfile
import pstats
import functools
import datetime
import os
from sensor_msgs.msg import JointState
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from my_robot_interfaces.srv import SetRotatedForbiddenBox
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose as GeoPose
from moveit_msgs.srv import ApplyPlanningScene
import numpy as np

def profile_this(func):
    @functools.wraps(func)  # <-- preserves original function name/signature for profiler
    def wrapper(*args, **kwargs):
        profiler = cProfile.Profile()
        profiler.enable()
        try:
            return func(*args, **kwargs)
        finally:
            profiler.disable()
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"profile_{func.__name__}_{timestamp}.stats"
            profiler.dump_stats(filename)
            print(f"[Profiler] Saved profiling data to {os.path.abspath(filename)}")

            stats = pstats.Stats(profiler).sort_stats('cumtime')
            print(f"[Profiler] {func.__name__} total time: {stats.total_tt:.4f}s")
    return wrapper

class MoveAR(Node):

    def __init__(self):
        super().__init__("move_ar")
        self.log_with_time('info' ,"\033[92mStarting Init\033[0m")

        # Create a secondary logger for file output
        self.file_logger = logging.getLogger('move_ar_logger')
        self.file_logger.setLevel(logging.INFO)

        # File handler setup
        file_handler = logging.FileHandler('/home/alon/Documents/move_ar.txt')
        file_handler.setLevel(logging.INFO)

        # Add a formatter for clear log structure
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)

        # Attach the handler to the new logger
        self.file_logger.addHandler(file_handler)
        self.shared_cb_group = ReentrantCallbackGroup()
        # Define callback group for service
        self.callback_group = ReentrantCallbackGroup()
        # Create service that listens for Pose messages
        self.srv = self.create_service(
            MoveToPose,  # Service type
            'ar_move_to_pose',  # Service name
            self.handle_move_ar,  # Callback function
            callback_group=self.shared_cb_group
        )
        
        self.srv_rotated_forbidden = self.create_service(
            SetRotatedForbiddenBox,
            "set_rotated_forbidden_box",
            self.handle_set_rotated_forbidden_box,
            callback_group=self.shared_cb_group
        )

        # --- Planning scene client ---
        self.scene_client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene'
        )

        self.current_board_object_id = None


        self.log_with_time('info', "Service 'ar_move_to' is ready to receive Pose messages.")

        self.arm_joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
        ]

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.arm_joint_names,
            base_link_name="base_link",
            end_effector_name="link_6",
            group_name="ar_manipulator",
            callback_group=self.shared_cb_group
        )
        self.moveit2.planner_id = "RRTConnectkConfigDefault"
        self.moveit2.max_velocity = 1.0
        self.moveit2.max_acceleration = 1.0
        self.moveit2.planning_time = 5.0  # Timeout in seconds

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._prev_marker_pose = None
        
        #Timer: callback every 2.0 seconds
        self.timer_callback_group = ReentrantCallbackGroup()
        # self.timer = self.create_timer(2.0, self.timer_callback,callback_group=self.shared_cb_group)
        self.last_timer_time = time.time()
        self._move_done_event = threading.Event()
        self._move_in_progress = False
    def _wait_for_scene_service(self):
        if not self.scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ apply_planning_scene service not available!")
            return False
        return True

    def _remove_old_board_object(self):
        if self.current_board_object_id is None:
            return

        co = CollisionObject()
        co.id = self.current_board_object_id
        co.header.frame_id = "base_link"
        co.operation = CollisionObject.REMOVE

        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects.append(co)

        req = ApplyPlanningScene.Request(scene=ps)
        self.scene_client.call_async(req)

        self.get_logger().info(f"🗑️ Removed old board collision: {self.current_board_object_id}")
        self.current_board_object_id = None

    def handle_set_rotated_forbidden_box(self, req, res):
        """
        Service callback: update the board collision box using p1, p2, height, depth.
        """

        p1 = (req.p1_x, req.p1_y)
        p2 = (req.p2_x, req.p2_y)

        try:
            self.update_rotated_board_collision(
                p1=p1,
                p2=p2,
                height=req.height,
                depth=req.depth
            )
            res.success = True
            res.message = "Updated rotated forbidden box."
        except Exception as e:
            res.success = False
            res.message = f"Failed: {e}"
            self.get_logger().error(res.message)

        return res



    def update_rotated_board_collision(self, p1, p2, height, depth):
        """
        Creates/updates a rotated collision box for the board.
        p1 and p2 define the XY span.
        height defines Z size.
        depth defines thickness of the box.
        """
        if not self._wait_for_scene_service():
            return

        # remove previous
        self._remove_old_board_object()

        x1, y1 = p1
        x2, y2 = p2

        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx*dx + dy*dy)

        yaw = math.atan2(dy, dx)

        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        cz = height / 2.0   # assumes bottom of board at Z=0

        # box primitive
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(length), float(depth), float(height)]

        q = tf_transformations.quaternion_from_euler(0, 0, yaw)

        pose = GeoPose()
        pose.position.x = cx
        pose.position.y = cy
        pose.position.z = cz
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        co = CollisionObject()
        co.header.frame_id = "base_link"
        co.id = "forbidden_board_box"
        co.operation = CollisionObject.ADD
        co.primitives = [box]
        co.primitive_poses = [pose]

        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects.append(co)

        req = ApplyPlanningScene.Request(scene=ps)
        self.scene_client.call_async(req)

        self.current_board_object_id = co.id
        self.get_logger().info(
            f"Updated rotated board box: center=({cx:.3f},{cy:.3f},{cz:.3f}), "
            f"yaw={math.degrees(yaw):.1f}°, L={length:.3f}, D={depth:.3f}, H={height:.3f}"
        )


    def joint_state_callback(self, msg):
        self.moveit2.last_joint_state = msg

    def log_with_time(self, level, message):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        full_message = f"[{timestamp}] {message}"
        if level == 'info':
            self.get_logger().info(full_message)
        elif level == 'error':
            self.get_logger().error(full_message)
        elif level == 'warn':
            self.get_logger().warn(full_message)
        else:
            self.get_logger().debug(full_message)

    def timer_callback(self):
        now = time.time()
        elapsed = now - self.last_timer_time
        self.last_timer_time = now

        self.log_with_time('info' ,f'ROS loop is alive! Elapsed: {elapsed:.3f} seconds')

    def write_to_logger(self, transformed_pose):
        # Extract position values
        position = transformed_pose.position
        x, y, z = round(position.x, 3), round(position.y, 3), round(position.z, 3)

           # Extract rotation (quaternion) values
        orientation = transformed_pose.orientation
        qx, qy, qz, qw = (
            round(orientation.x, 3),
            round(orientation.y, 3),
            round(orientation.z, 3),
            round(orientation.w, 3)
        )

        # Log the formatted position
        self.file_logger.info(f"Position: {x},{y},{z} | Rotation: {qx},{qy},{qz},{qw}")
        

    def MoveArmWithViaPoints(self, via_points, final_position, final_quat):
        """Execute ONE continuous Cartesian trajectory through all via points."""
        return self.MoveArmCartesian(final_position, final_quat, waypoints=via_points)



    def handle_move_ar(self, request: MoveToPose.Request, response: MoveToPose.Response):
        """
        Callback for the 'ar_move_to_pose' service:
        - Supports optional via-points (geometry_msgs/Pose[])
        - Moves through via-points (if any), then to final pose
        - Supports both Cartesian and Joint planning modes
        """

        pose_to_move = request.pose
        cartesian = request.cartesian
        via_points = request.via_points  # <-- new field from updated .srv

        timeout = 230  # seconds
        start_time = time.time()

        self.moveit2.set_is_executing(True)
        planning_success = False

        # ---------------------------------------------------------
        # CASE 1: There are via points
        # ---------------------------------------------------------
        if len(via_points) > 0:
            self.log_with_time('info', f"Received {len(via_points)} via point(s). Planning through them...")

            planning_success = self.MoveArmWithViaPoints(
                via_points=via_points,
                final_position=pose_to_move.position,
                final_quat=pose_to_move.orientation,
                # cartesian=cartesian
            )

        # ---------------------------------------------------------
        # CASE 2: No via points - normal behavior
        # ---------------------------------------------------------
        else:
            if cartesian:
                print("Moving in Cartesian space")
                planning_success = self.MoveArmCartesian(
                    pose_to_move.position,
                    pose_to_move.orientation
                )
            else:
                print("Moving in joint space")
                planning_success = self.MoveArm(
                    pose_to_move.position,
                    pose_to_move.orientation
                )

        # ---------------------------------------------------------
        # If planning failed entirely
        # ---------------------------------------------------------
        if not planning_success:
            response.success = False
            response.message = "Planning failed"
            self.log_with_time('warn', "Planning failed; returning early without execution.")
            return response

        # ---------------------------------------------------------
        # Wait for execution to finish (same as before)
        # ---------------------------------------------------------
        while self.moveit2.is_executing():
            elapsed = time.time() - start_time
            if elapsed > timeout:
                response.success = False
                response.message = "Timeout reached"
                self.log_with_time('error', "Execution timeout reached; aborting motion.")
                return response
            time.sleep(0.05)

        # ---------------------------------------------------------
        # Final Result
        # ---------------------------------------------------------
        response.success = self.moveit2.motion_suceeded
        response.message = "Motion completed" if self.moveit2.motion_suceeded else "Motion failed"

        return response

    def _log_motion_to_file(self, motion_type, success, start_pose=None, target_pose=None):
        log_path = os.path.expanduser("~/arm_motion.log")
        now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        def fmt(pose):
            if not pose:
                return "None"
            if isinstance(pose, list):
                pose = pose[0]
            if hasattr(pose, 'pose'):
                pos = pose.pose.position
                ori = pose.pose.orientation
                return f"pos=({pos.x:.3f},{pos.y:.3f},{pos.z:.3f}) ori=({ori.x:.3f},{ori.y:.3f},{ori.z:.3f},{ori.w:.3f})"
            elif isinstance(pose, Pose):
                pos = pose.position
                ori = pose.orientation
                return f"pos=({pos.x:.3f},{pos.y:.3f},{pos.z:.3f}) ori=({ori.x:.3f},{ori.y:.3f},{ori.z:.3f},{ori.w:.3f})"
            elif isinstance(pose, Point):
                return f"pos=({pose.x:.3f},{pose.y:.3f},{pose.z:.3f})"
            else:
                return f"UnknownType({type(pose)})"
        
        log_entry = f"[{now}] Motion: {motion_type}, Success: {success}, Start: {fmt(start_pose)},  Target: {fmt(target_pose)}\n"
        with open(log_path, "a") as f:
            f.write(log_entry)


    def _interpolate_pose(self, pose1, pose2, alpha=0.5):
        """Linear interpolation of two poses (position only, keeps orientation of pose1)."""
        p1 = pose1.pose.position
        p2 = pose2.pose.position
        mid = PoseStamped()
        mid.header.frame_id = "base_link"
        mid.pose.position.x = (1 - alpha) * p1.x + alpha * p2.x
        mid.pose.position.y = (1 - alpha) * p1.y + alpha * p2.y
        mid.pose.position.z = (1 - alpha) * p1.z + alpha * p2.z
        mid.pose.orientation = pose1.pose.orientation  # keep start orientation for simplicity
        return mid

    def _pose_distance(self, pose1, pose2):
        """Euclidean distance between two positions."""
        p1 = pose1.pose.position
        p2 = pose2.pose.position
        return ((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2) ** 0.5

    def _pose_to_str(self, pose):
        p = pose.pose.position
        return f"({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
    def _midpoint(self, p1, p2):
        """Return the midpoint between two geometry_msgs.msg.Point objects."""
        return Point(
            x=(p1.x + p2.x) / 2.0,
            y=(p1.y + p2.y) / 2.0,
            z=(p1.z + p2.z) / 2.0,
        )

    def MoveArm(self, position, quat_xyzw):
            """Plans and executes a joint-space move to the target pose"""
            # time.sleep(0.5)  # Allow time for arm/controller to become ready

            # self.log_with_time("info","Starting joint-space motion...")

            # build the final target pose
            target_pose = PoseStamped()
            target_pose.header.frame_id = "base_link"
            target_pose.pose = Pose(position=position, orientation=quat_xyzw)

            # now do the normal move to the final pose
            try:
                success = self.moveit2.move_to_pose(
                    pose=target_pose,
                    cartesian=False
                )
            except Exception as e:
                self.log_with_time("error",f"Joint-space planning to final target failed: {e}")
                return False

            if not success:
                self.log_with_time("error","Joint-space planning failed; aborting motion.")
                return False

            # self.log_with_time("info","Joint-space motion planned and executed successfully.")
            print("Joint-space motion planned and executed successfully.")
            return True


    def _build_pose(self, position, quat):
        """
        Helper: create a Pose from:
          - position: geometry_msgs.msg.Point OR (x, y, z)
          - quat: geometry_msgs.msg.Quaternion OR (x, y, z, w)
        """
        pose = Pose()

        # Position
        if isinstance(position, Point):
            pose.position = position
        else:
            # assume tuple/list
            pose.position.x = position[0]
            pose.position.y = position[1]
            pose.position.z = position[2]

        # Orientation
        if isinstance(quat, Quaternion):
            pose.orientation = quat
        else:
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

        return pose

    def _normalize_waypoint(self, wp, default_orientation):
        """
        Helper: convert different waypoint formats into a Pose.

        wp can be:
          - Pose
          - Point
          - (x, y, z) tuple/list

        default_orientation: Quaternion to use when wp has no orientation.
        """
        if isinstance(wp, Pose):
            return wp

        pose_wp = Pose()

        if isinstance(wp, Point):
            pose_wp.position = wp
        else:
            # assume tuple/list
            pose_wp.position.x = wp[0]
            pose_wp.position.y = wp[1]
            pose_wp.position.z = wp[2]

        pose_wp.orientation = default_orientation
        return pose_wp
    def MoveArmCartesian(self, final_position, final_quat, waypoints=None):
        """
        Move the arm to a target pose, optionally passing through via-points.

        final_position: geometry_msgs.msg.Point OR (x, y, z)
        final_quat: geometry_msgs.msg.Quaternion OR (x, y, z, w)
        waypoints: list of Pose / Point / (x, y, z)
        """

        # Build final pose robustly
        final_pose = self._build_pose(final_position, final_quat)

        # ---- No via-points: simple move_to_pose ----
        if not waypoints:
            success = self.moveit2.move_to_pose(
                pose=final_pose,
                cartesian=False
            )
            if not success:
                self.get_logger().error("MoveArmCartesian: failed to reach final pose.")
            return success

        # ---- With via-points: step through them ----
        for idx, wp in enumerate(waypoints):
            pose_wp = self._normalize_waypoint(wp, final_pose.orientation)

            ok = self.moveit2.move_to_pose(
                pose=pose_wp,
                cartesian=False
            )
            if not ok:
                self.get_logger().error(
                    f"MoveArmCartesian: failed to reach via-point #{idx}."
                )
                return False

        # Finally go to the target
        success = self.moveit2.move_to_pose(
            pose=final_pose,
            cartesian=False
        )
        if not success:
            self.get_logger().error("MoveArmCartesian: failed to reach final pose after via-points.")
        return success


    def MoveArmCartesianOld(self, position, quat_xyzw):
        """Plans and executes a Cartesian-space move to the target pose."""
        time.sleep(0.5)  # Allow time for arm/controller to become ready

        self.log_with_time('info' ,"Starting Cartesian motion...")

        try:
            self.log_with_time('info' ,f"cartesian_max_step: 0.01, cartesian_fraction_threshold: 0.2")
            success = self.moveit2.move_to_pose(
                position=position,
                quat_xyzw=quat_xyzw,
                cartesian=True,
                # cartesian_max_step=0.0025,           # Step size in meters
                cartesian_max_step=0.01,           # Step size in meters
                cartesian_fraction_threshold=0.2     # Minimum acceptable fraction
            )
        except Exception as e:
            self.log_with_time('error' ,f"Cartesian planning failed with exception: {e}")
            self._log_motion_to_file("Cartesian", False, None, position)

            return False

        if not success:
            self.log_with_time('error' ,"Cartesian planning failed; aborting motion.")
            self._log_motion_to_file("Cartesian", False, None, position)

            return False

        self.log_with_time('info' ,"Cartesian motion planned successfully")
        self._log_motion_to_file("Cartesian", True, None, position)

        return True


def main():
    # debugpy.listen(("0.0.0.0", 5678))
    # print("Waiting for debugger to attach...")
    # debugpy.wait_for_client()  # Uncomment this if you want to pause execution until the debugger attaches
    # print("debugger attached...")
    # print("🔥 THIS IS THE NEW CODE")
    
    rclpy.init()
    
     # Create a temporary node to query the ROS graph
    tmp_node = rclpy.create_node('_node_checker')
    existing_nodes = tmp_node.get_node_names()
    tmp_node.destroy_node()

    print("Existing nodes in the ROS graph:")
    move_ar_count = 0
    for n in existing_nodes:
        print(f" - {n}")
        if n.lstrip('/').startswith('move_ar'):
            move_ar_count += 1

    if move_ar_count > 1:
        print(f"[ERROR] Detected {move_ar_count} 'move_ar' nodes already running. Exiting.")
        rclpy.shutdown()
        return
    else:
        print(f"[INFO] Detected {move_ar_count} existing 'move_ar' node(s) (including self). Continuing...")
        
    print(f"[DEBUG] pymoveit2 loaded from: {pymoveit2.__file__}")

    node = MoveAR()
    executor = MultiThreadedExecutor(20) 
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":


    main()
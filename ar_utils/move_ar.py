#!/usr/bin/env python3
"""
A script to follow an aruco marker with a robot arm using PyMoveit2.
"""
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from my_robot_interfaces.srv import MoveToPose  # Import the custom service type
from geometry_msgs.msg import Pose 
from geometry_msgs.msg import Point 
# from moveit_commander.action import MoveGroupAction, MoveGroupGoal
from moveit_msgs.action import MoveGroup
import threading
import sys
import tf2_ros
from geometry_msgs.msg import PoseStamped
from typing import Optional
import os
import datetime

from geometry_msgs.msg import Pose,Vector3
#from tf2_geometry_msgs import do_transform_pose
sys.path.insert(0, '/home/alon/ros_ws/src/pymoveit2')

from pymoveit2 import MoveIt2
import pymoveit2



import logging
import debugpy
import atexit
import time
from pymoveit2.moveit2 import init_execute_trajectory_goal



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, WorkspaceParameters
from shape_msgs.msg import SolidPrimitive
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
import threading


#view end effector position
#ros2 run tf2_ros tf2_echo base_link ee_link


        
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
        
        self.log_with_time('info' ,"\033[92mService 'ar_move_to' is ready to receive Pose messages.\033[0m")


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


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._prev_marker_pose = None
        
        #Timer: callback every 2.0 seconds
        self.timer_callback_group = ReentrantCallbackGroup()
        # self.timer = self.create_timer(2.0, self.timer_callback,callback_group=self.shared_cb_group)
        self.last_timer_time = time.time()
        self._move_done_event = threading.Event()
        self._move_in_progress = False

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
        

    def handle_move_ar(self, request: MoveToPose.Request, response: MoveToPose.Response):
        """Callback for the 'ar_move_to' service: receives a pose, plans and executes the move, and responds."""
        self.log_with_time('info' ,f"Received move request: Position ({request.pose.position.x}, {request.pose.position.y}, {request.pose.position.z})")
        self.log_with_time('info' ,f"Orientation: ({request.pose.orientation.x}, {request.pose.orientation.y}, {request.pose.orientation.z}, {request.pose.orientation.w})")

        pose_to_move = request.pose
        cartesian = request.cartesian

        self.log_with_time('info' ,f"Moving to: {pose_to_move}")

        timeout = 230  # seconds
        start_time = time.time()

        planning_success = False
        self.moveit2.set_is_executing(True)

        if cartesian:
            self.log_with_time('info' ,"Moving in Cartesian space")
            planning_success = self.MoveArmCartesian(pose_to_move.position, pose_to_move.orientation)
        else:
            self.log_with_time('info' ,"Moving in joint space")
            planning_success = self.MoveArm(pose_to_move.position, pose_to_move.orientation)

        if not planning_success:
            response.success = False
            response.message = "Planning failed"
            self.log_with_time('warn' ,"Planning failed; returning early without execution.")
            return response

        # If planning succeeded, wait for execution to complete
        self.log_with_time('info' ,"Waiting for motion to complete...")
        while self.moveit2.is_executing():
            elapsed = time.time() - start_time
            # self.log_with_time('info' ,f"Still waiting for motion to complete... elapsed {elapsed:.1f}s")
            if elapsed > timeout:
                response.success = False
                response.message = "Timeout reached"
                self.log_with_time('error' ,"Execution timeout reached; aborting motion.")
                return response
            time.sleep(0.05)

        self.log_with_time('info' ,"Motion completed")
        response.success = self.moveit2.motion_suceeded
        response.message = "Motion completed" if self.moveit2.motion_suceeded else "Motion failed"

        self.log_with_time('info' ,f"Move result: {response.success}, Message: {response.message}")
        return response


    def MoveArmOld(self, position, quat_xyzw):
        """Plans and executes a joint-space move to the target pose."""
        time.sleep(0.5)  # Allow time for arm/controller to become ready

        self.log_with_time('info' ,"Starting joint-space motion...")

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose = Pose(position=position, orientation=quat_xyzw)

        try:
            success = self.moveit2.move_to_pose(
                pose=pose_goal,
                cartesian=False
            )
        except Exception as e:
            self.log_with_time('error' ,f"Joint-space planning failed: {e}")
            return False

        if not success:
            self.log_with_time('error' ,f"Joint-space planning failed; aborting motion. success: {success}")
            return False

        self.log_with_time('info' ,"Joint-space motion planned successfully, sucess:" + str(success))
        return True

    def _on_via_point_found(self, via_pose: Optional[PoseStamped]):
        if via_pose is None:
            self.log_with_time('warn' ,"No valid via-point found. Attempting direct move to goal.")
            # Try direct move
            # self.moveit2.move_to_pose_async(
            #     pose=self.goal_pose,
            #     callback=self._on_motion_done,
            #     path_constraints=self._move_action_goal.request.path_constraints
            #         if hasattr(self, "_move_action_goal") else None
            # )
            return

        self.log_with_time('info' ,"Via-point found. Moving to via-point first.")

        # First move to the via-point
        # self.moveit2.move_to_pose_async(
        #     pose=via_pose,
        #     callback=self._on_via_reached
        # )



    def _on_start_pose_ready(self, future):
        try:
            poses = future.result()
            start_pose = poses[0]  # assuming only one link requested
            self.start_pose = start_pose
            self.log_with_time('info' ,f"Got FK pose: {start_pose}")
            
            # Now continue with whatever needs this pose
            self.find_via_point(
                start=start_pose,
                goal=self.goal_pose,
                steps=20,
                callback=self._on_via_point_found
            )
        except Exception as e:
            self.log_with_time('error' ,f"Failed to get FK: {e}")



    def get_start_pose_async(self):
        future = self.moveit2.compute_fk_async(
            joint_state=self.moveit2.joint_state,
            fk_link_names=[self.moveit2.end_effector_name]
        )
        future.add_done_callback(self._on_start_pose_ready)


    def _log_motion_to_file(self, motion_type, success, start_pose=None, via_pose=None, target_pose=None):
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
        
        log_entry = f"[{now}] Motion: {motion_type}, Success: {success}, Start: {fmt(start_pose)}, Via: {fmt(via_pose)}, Target: {fmt(target_pose)}\n"
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
        """Plans and executes a joint-space move to the target pose, subdividing as needed to avoid planning failures."""
        time.sleep(0.5)  # Allow time for arm/controller to become ready

        self.log_with_time('info' ,"Starting joint-space motion...")

        # Build target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose = Pose(position=position, orientation=quat_xyzw)

        # Get current FK pose
        start_fk = self.moveit2.compute_fk(
            joint_state=self.moveit2.joint_state,
            fk_link_names=[self.moveit2.end_effector_name]
        )
        if not start_fk:
            self.log_with_time('error' ,"FK failed, cannot compute start pose.")
            self._log_motion_to_file("Joint-space", False, None, None, target_pose)
            return False
        current_pose = start_fk[0]

        # Stack for path segments (LIFO)
        path_stack = [(current_pose, target_pose)]
        successful_path = []
        max_depth = 10
        depth = 0

        while path_stack and depth < max_depth:
            from_pose, to_pose = path_stack.pop()
            self.log_with_time('info' ,f"Planning from {self._pose_to_str(from_pose)} to {self._pose_to_str(to_pose)}")

            try:
                success = self.moveit2.move_to_pose(pose=to_pose, cartesian=False)
            except Exception as e:
                self.log_with_time('error' ,f"Planning failed with exception: {e}")
                success = False

            if success:
                self.log_with_time('info' ,f"Motion succeeded to {self._pose_to_str(to_pose)}")
                successful_path.append(to_pose)

                # Compute FK to get new robot pose
                fk_result = self.moveit2.compute_fk(
                    joint_state=self.moveit2.joint_state,
                    fk_link_names=[self.moveit2.end_effector_name]
                )
                if not fk_result:
                    self.log_with_time('error' ,"FK failed after motion; aborting.")
                    self._log_motion_to_file("Joint-space", False, None, successful_path, target_pose)
                    return False

                new_pose = fk_result[0]

                # Check if target is already reached (within small threshold)
                if self._pose_distance(new_pose, target_pose) < 0.01:
                    self.log_with_time('info' ,"Target reached successfully.")
                    self._log_motion_to_file("Joint-space", True, current_pose, successful_path, target_pose)
                    return True

                # Not yet there; push remaining segment
                path_stack.append((new_pose, target_pose))
                continue

            else:
                # Planning failed — try to subdivide
                self.log_with_time('warn' ,"Motion failed, subdividing...")

                mid_pose = PoseStamped()
                mid_pose.header.frame_id = "base_link"
                mid_pose.pose.position = self._midpoint(from_pose.pose.position, to_pose.pose.position)
                mid_pose.pose.orientation = to_pose.pose.orientation  # Use target orientation
                self.log_with_time('info' ,f"Subdividing path: {self._pose_to_str(from_pose)} to {self._pose_to_str(mid_pose)} and {self._pose_to_str(mid_pose)} to {self._pose_to_str(to_pose)}")    
                path_stack.append((mid_pose, to_pose))
                path_stack.append((from_pose, mid_pose))

            depth += 1

        # Failed to reach target
        self.log_with_time('error' ,"Failed to reach final target after subdivisions.")
        self._log_motion_to_file("Joint-space", False, current_pose, successful_path, target_pose)
        return False



    def MoveArmbeforesubdevision(self, position, quat_xyzw):
        """Plans and executes a joint-space move to the target pose, via an automatically computed via‐point."""
        time.sleep(0.5)  # Allow time for arm/controller to become ready

        self.log_with_time('info' ,"Starting joint-space motion...")

        # build the final target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose = Pose(position=position, orientation=quat_xyzw)

        # Compute current start pose from FK
        start_pose = self.moveit2.compute_fk(
            joint_state=self.moveit2.joint_state,
            fk_link_names=[self.moveit2.end_effector_name]
        )
        if not start_pose:
            self.log_with_time('error' ,"FK failed, cannot compute via-point.")
            self._log_motion_to_file("Joint-space", False, start_pose, via_pose, target_pose)

            return False

        # --- BLOCKING via-point computation ---
        via_point_event = threading.Event()
        via_point_result = {"pose": None}

        def _on_via_point_found(pose):
            via_point_result["pose"] = pose
            via_point_event.set()

        self.moveit2.find_via_point(start=start_pose, goal=target_pose, callback=_on_via_point_found)

        if not via_point_event.wait(timeout=11112.0):  # Adjust timeout as needed
            self.log_with_time('warn' ,"Timed out waiting for via-point computation.")
            self._log_motion_to_file("Joint-space", False, start_pose, via_pose, target_pose)

            return False

        via_pose = via_point_result["pose"]
        if via_pose:
            try:
                self.log_with_time('info' ,f"Moving to via-point: {via_pose.pose.position}")
                success = self.moveit2.move_to_pose(pose=via_pose, cartesian=False)
            except Exception as e:
                self.log_with_time('error' ,f"Joint-space planning to via-point failed: {e}")
                self._log_motion_to_file("Joint-space", False, start_pose, via_pose, target_pose)

                return False

            if not success:
                self.log_with_time('error' ,f"Failed to reach via-point; aborting motion.")
                self._log_motion_to_file("Joint-space", False, start_pose, via_pose, target_pose)
                return False

            self.log_with_time('info' ,"Reached via-point successfully; now moving to final target.")

        # now do the normal move to the final pose
        try:
            self.log_with_time('info' ,f"Moving to final target: {target_pose.pose.position}")
            success = self.moveit2.move_to_pose(pose=target_pose, cartesian=False)
        except Exception as e:
            self.log_with_time('error' ,f"Joint-space planning to final target failed: {e}")
            self._log_motion_to_file("Joint-space", False, start_pose, via_pose, target_pose)
            return False

        # if not success:
        #     self.log_with_time('error' ,"Joint-space planning failed; aborting motion.")
        #     return False

        self.log_with_time('info' ,"Joint-space motion planned and executed successfully.")
        self._log_motion_to_file("Joint-space", True, start_pose, via_pose, target_pose)

        return True


    def MoveArmCartesian(self, position, quat_xyzw):
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
            self._log_motion_to_file("Cartesian", False, None, None, position)

            return False

        if not success:
            self.log_with_time('error' ,"Cartesian planning failed; aborting motion.")
            self._log_motion_to_file("Cartesian", False, None, None, position)

            return False

        self.log_with_time('info' ,"Cartesian motion planned successfully")
        self._log_motion_to_file("Cartesian", True, None, None, position)

        return True


def main():
    # debugpy.listen(("0.0.0.0", 5678))
    # print("Waiting for debugger to attach...")
    # debugpy.wait_for_client()  # Uncomment this if you want to pause execution until the debugger attaches
    # print("debugger attached...")

    
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
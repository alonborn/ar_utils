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

import tf2_ros
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose,Vector3
#from tf2_geometry_msgs import do_transform_pose
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
        self.get_logger().info("\033[92mStarting Init\033[0m")
        self.logger = self.get_logger()

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
        
        self.get_logger().info("\033[92mService 'ar_move_to' is ready to receive Pose messages.\033[0m")


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
    
    def timer_callback(self):
        now = time.time()
        elapsed = now - self.last_timer_time
        self.last_timer_time = now

        self.get_logger().info(f'ROS loop is alive! Elapsed: {elapsed:.3f} seconds')

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
        self.get_logger().info(f"Received move request: Position ({request.pose.position.x}, {request.pose.position.y}, {request.pose.position.z})")
        self.get_logger().info(f"Orientation: ({request.pose.orientation.x}, {request.pose.orientation.y}, {request.pose.orientation.z}, {request.pose.orientation.w})")

        pose_to_move = request.pose
        cartesian = request.cartesian

        self.logger.info(f"Moving to: {pose_to_move}")

        timeout = 30  # seconds
        start_time = time.time()

        planning_success = False
        self.moveit2.set_is_executing(True)

        if cartesian:
            self.logger.info("Moving in Cartesian space")
            planning_success = self.MoveArmCartesian(pose_to_move.position, pose_to_move.orientation)
        else:
            self.logger.info("Moving in joint space")
            planning_success = self.MoveArm(pose_to_move.position, pose_to_move.orientation)

        if not planning_success:
            response.success = False
            response.message = "Planning failed"
            self.logger.warn("Planning failed; returning early without execution.")
            return response

        # If planning succeeded, wait for execution to complete
        self.get_logger().info("Waiting for motion to complete...")
        while self.moveit2.is_executing():
            elapsed = time.time() - start_time
            self.get_logger().info(f"Still waiting for motion to complete... elapsed {elapsed:.1f}s")
            if elapsed > timeout:
                response.success = False
                response.message = "Timeout reached"
                self.logger.error("Execution timeout reached; aborting motion.")
                return response
            time.sleep(0.05)

        self.get_logger().info("Motion completed")
        response.success = self.moveit2.motion_suceeded
        response.message = "Motion completed" if self.moveit2.motion_suceeded else "Motion failed"

        self.get_logger().info(f"Move result: {response.success}, Message: {response.message}")
        return response


    def MoveArmOld(self, position, quat_xyzw):
        """Plans and executes a joint-space move to the target pose."""
        time.sleep(0.5)  # Allow time for arm/controller to become ready

        self.logger.info("Starting joint-space motion...")

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose = Pose(position=position, orientation=quat_xyzw)

        try:
            success = self.moveit2.move_to_pose(
                pose=pose_goal,
                cartesian=False
            )
        except Exception as e:
            self.logger.error(f"Joint-space planning failed: {e}")
            return False

        if not success:
            self.logger.error(f"Joint-space planning failed; aborting motion. success: {success}")
            return False

        self.logger.info("Joint-space motion planned successfully, sucess:" + str(success))
        return True

    def MoveArm(self, position, quat_xyzw):
        """Plans and executes a joint-space move to the target pose, via an automatically computed via‐point."""
        time.sleep(0.5)  # Allow time for arm/controller to become ready

        self.logger.info("Starting joint-space motion...")

        # build the final target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose = Pose(position=position, orientation=quat_xyzw)

        # ← **NEW** compute an intermediate via‐point to avoid flips/singularities
        try:
            via_pose = self.moveit2.find_via_point(target_pose)
            self.logger.info(f"Via-point computed: {via_pose.pose.position}")
        except Exception as e:
            self.logger.warn(f"Via-point computation failed ({e}); will try direct move only.")
            via_pose = None

        # ← **NEW** if we got a via‐point, move there first
        if via_pose is not None:
            try:
                success = self.moveit2.move_to_pose(
                    pose=via_pose,
                    cartesian=False
                )
            except Exception as e:
                self.logger.error(f"Joint-space planning to via-point failed: {e}")
                return False

            if not success:
                self.logger.error(f"Failed to reach via-point; aborting motion.")
                return False

            self.logger.info("Reached via-point successfully; now moving to final target.")

        # now do the normal move to the final pose
        try:
            success = self.moveit2.move_to_pose(
                pose=target_pose,
                cartesian=False
            )
        except Exception as e:
            self.logger.error(f"Joint-space planning to final target failed: {e}")
            return False

        if not success:
            self.logger.error("Joint-space planning failed; aborting motion.")
            return False

        self.logger.info("Joint-space motion planned and executed successfully.")
        return True




    def MoveArmCartesian(self, position, quat_xyzw):
        """Plans and executes a Cartesian-space move to the target pose."""
        time.sleep(0.5)  # Allow time for arm/controller to become ready

        self.logger.info("Starting Cartesian motion...")

        try:
            self.get_logger().info(f"cartesian_max_step: 0.01, cartesian_fraction_threshold: 0.2")
            success = self.moveit2.move_to_pose(
                position=position,
                quat_xyzw=quat_xyzw,
                cartesian=True,
                # cartesian_max_step=0.0025,           # Step size in meters
                cartesian_max_step=0.01,           # Step size in meters
                cartesian_fraction_threshold=0.2     # Minimum acceptable fraction
            )
        except Exception as e:
            self.logger.error(f"Cartesian planning failed with exception: {e}")
            return False

        if not success:
            self.logger.error("Cartesian planning failed; aborting motion.")
            return False

        self.logger.info("Cartesian motion planned successfully")
        return True


def main():
    debugpy.listen(("0.0.0.0", 5678))
    print("Waiting for debugger to attach...")
    debugpy.wait_for_client()  # Uncomment this if you want to pause execution until the debugger attaches
    print("debugger attached...")

    
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
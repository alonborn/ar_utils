#!/usr/bin/env python3
"""
A script to follow an aruco marker with a robot arm using PyMoveit2.
"""
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from my_robot_interfaces.srv import MoveToPose  # Import the custom service type
from geometry_msgs.msg import Pose
# from moveit_commander.action import MoveGroupAction, MoveGroupGoal
from moveit_msgs.action import MoveGroup

import tf2_ros
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from tf2_geometry_msgs import do_transform_pose
from pymoveit2 import MoveIt2
import logging
import debugpy

import time

#view end effector position
#ros2 run tf2_ros tf2_echo base_link ee_link

class MoveAR(Node):

    def __init__(self):
        super().__init__("move_ar")
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

        # Define callback group for service
        self.callback_group = MutuallyExclusiveCallbackGroup()
        
        # Create service that listens for Pose messages
        self.srv = self.create_service(
            MoveToPose,  # Service type
            'ar_move_to_pose',  # Service name
            self.handle_move_ar  # Callback function
        )
        
        self.get_logger().info("\033[92mService 'ar_move_to' is ready to receive Pose messages.\033[0m")


        self.arm_joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
        ]


        # Create an action client for MoveGroup
        # self.client = self.create_client(MoveGroupAction, '/move_group')
        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service /move_group not available, waiting again...')

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.arm_joint_names,
            base_link_name="base_link",
            end_effector_name="link_6",
            group_name="ar_manipulator",
            callback_group=ReentrantCallbackGroup(),
        )
        self.moveit2.planner_id = "RRTConnectkConfigDefault"
        self.moveit2.max_velocity = 1.0
        self.moveit2.max_acceleration = 1.0
        self.moveit2.planning_time = 5.0  # Timeout in seconds


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._prev_marker_pose = None


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
        

    def handle_move_ar(self, request: MoveToPose.Request, response:MoveToPose.Response):
        
        """Callback function for the 'ar_move_to' service."""
        self.get_logger().info(f"Received move request: Position ({request.pose.position.x}, {request.pose.position.y}, {request.pose.position.z})")
        self.get_logger().info(f"Orientation: ({request.pose.orientation.x}, {request.pose.orientation.y}, {request.pose.orientation.z}, {request.pose.orientation.w})")
       
        # pose_to_move = Pose()
        # pose_to_move.position.x = 0.04
        # pose_to_move.position.y = -0.31
        # pose_to_move.position.z = 0.375
        # pose_to_move.orientation.x = 0.044
        # pose_to_move.orientation.y = -0.702
        # pose_to_move.orientation.z = 0.71
        # pose_to_move.orientation.w = -0.033

        pose_to_move = request.pose

        self.logger.info(f"moving to: {pose_to_move}")
        time.sleep(3)
        #self.move_to(pose_to_move)
        self.logger.info(f"Done moving: {pose_to_move}")

        response.status = True
        response.message = "ok"
        return response

    def _transform_pose(self, pose: Pose, source_frame,
                        target_frame: str) -> Pose:
        # Get the transform from source frame to target frame
        transform = self.tf_buffer.lookup_transform(target_frame, source_frame,
                                                    Time())
        # Transform the pose
        transformed_pose = do_transform_pose(pose, transform)
        # publish pose
        stamped_pose = PoseStamped()
        stamped_pose.header.frame_id = target_frame
        stamped_pose.pose = transformed_pose
        self.pose_pub.publish(stamped_pose)

        pose.position.z += 0.05
        transformed_pose = do_transform_pose(pose, transform)

        stamped_pose = PoseStamped()
        stamped_pose.header.frame_id = target_frame
        stamped_pose.pose = transformed_pose
        self.target_pose_pub.publish(stamped_pose)
        return transformed_pose

    def move_to(self, msg: Pose):
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose = msg

        self.moveit2.move_to_pose(pose=pose_goal)
        ret = self.moveit2.wait_until_executed()
        print (ret)
        if ret:
            self.file_logger.info("Move successful")


def main():

    # Allow attaching the debugger remotely on port 5678
    # debugpy.listen(("0.0.0.0", 5678))
    # print("Waiting for debugger to attach...")
    # debugpy.wait_for_client()  # Uncomment this if you want to pause execution until the debugger attaches

    rclpy.init()
    node = MoveAR()
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
from rclpy.action import ActionServer
from my_robot_interfaces.action import MoveToPoseAc
import asyncio
from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import panda as robot  # You can replace this with your robot config
import time
import debugpy

class PoseGoalNode(Node):
    def __init__(self):
        super().__init__('pose_goal_node')

        #Timer: callback every 2.0 seconds
        self.timer_callback_group = ReentrantCallbackGroup()
        #self.timer_callback_group = self.callback_group
        self.timer = self.create_timer(2.0, self.timer_callback,callback_group=self.timer_callback_group)

        self.init_moveit()

    def init_moveit(self):
        self.arm_joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
        ]
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.arm_joint_names,
            base_link_name="base_link",
            end_effector_name="link_6",
            group_name="ar_manipulator",
            callback_group=self.callback_group
        )

        self.moveit2.planner_id = "RRTConnectkConfigDefault"
        self.moveit2.max_velocity = 1.0
        self.moveit2.max_acceleration = 1.0
        self.moveit2.planning_time = 5.0  # Timeout in seconds

        # Scale down velocity and acceleration of joints (percentage of maximum)
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5
        self.moveit2.cartesian_avoid_collisions = False
        self.moveit2.cartesian_jump_threshold = 0.0

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        pose = goal_handle.request.pose

        position = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
        )
        quat = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        self.moveit2.move_to_pose(position = position, quat_xyzw = quat)
        while self.moveit2.query_state() != MoveIt2State.EXECUTING:
            #await asyncio.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.1)
             

        future = self.moveit2.get_execution_future()

        while not future.done():
            # You can also compute progress here
            goal_handle.publish_feedback(MoveToPoseAc.Feedback(progress_percentage=50.0))
            rclpy.spin_once(self, timeout_sec=0.1)

        result = future.result().result
        goal_handle.succeed()
        # while True:
            # rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('cone operation...') 
        return MoveToPoseAc.Result(
            success=result.error_code.val == 1,
            error_code=result.error_code.val,
            message="Done" if result.error_code.val == 1 else "Failed"
        )


    def timer_callback(self):
        self.get_logger().info('ROS loop is alive1!')

    def handle_move_ar(self, rgoal_handle):
        """Callback function for the 'ar_move_to' service."""
        pass
        # self.get_logger().info(f"Received move request: Position ({request.pose.position}, Orientation {request.pose.orientation})")
        # self.execute_pose_goal(request.pose)
        # response.success = True  # You must respond
        # return response
        
    def execute_pose_goal(self,pose):
        # position = (0.03, -0.38, 0.39)
        # quat_xyzw = (0.0, 0.7071, 0.0, 0.7071)

        position = pose.position
        quat_xyzw = pose.orientation
    
        #cancel_after_secs = 50.0

        self.get_logger().info(f"Moving to pose: {position}, quat: {quat_xyzw}")

        # Send goal
        self.moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw)

        # Wait for execution to start
        while self.moveit2.query_state() != MoveIt2State.EXECUTING:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Get future and attach callback
        future = self.moveit2.get_execution_future()
        future.add_done_callback(self._on_motion_done)

        # Optional: cancel after time
        # if cancel_after_secs > 0.0:
        #     self.create_timer(
        #         cancel_after_secs, self._cancel_motion, callback_group=self.callback_group
        #     )

    def _on_motion_done(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"Execution result status: {result.status}")
            self.get_logger().info(f"Execution result error code: {result.result.error_code}")
        except Exception as e:
            self.get_logger().error(f"Execution failed: {e}")

    def _cancel_motion(self):
        self.get_logger().warn("Cancelling motion execution after timeout...")
        self.moveit2.cancel_execution()
def main():

    # debugpy.listen(("localhost", 5678))  # Port for debugger to connect
    # print("Waiting for debugger to attach...")
    # debugpy.wait_for_client()  # Ensures the debugger connects before continuing
    # print("Debugger connected.")

    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    node = PoseGoalNode()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

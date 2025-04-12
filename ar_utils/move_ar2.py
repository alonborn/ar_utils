#!/usr/bin/env python3

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from my_robot_interfaces.srv import MoveToPose  # Import the custom service type
from geometry_msgs.msg import Point 
from moveit_msgs.action import MoveGroup

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose,Vector3
import debugpy

from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, WorkspaceParameters
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
import threading


#view end effector position
#ros2 run tf2_ros tf2_echo base_link ee_link

class MoveAR(Node):
    def __init__(self):
        super().__init__('move_ar')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')  # Default MoveGroup interface
        self._move_in_progress = False
        self._move_done_event = threading.Event()

        self.callback_group = ReentrantCallbackGroup()
        # Create service that listens for Pose messages
        self.srv = self.create_service(
            MoveToPose,  # Service type
            'ar_move_to_pose',  # Service name
            self.handle_move_ar,  # Callback function
            callback_group=self.callback_group
        )

        #Timer: callback every 2.0 seconds
        self.timer_callback_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(2.0, self.timer_callback,callback_group=self.timer_callback_group)

    def timer_callback(self):
        self.get_logger().info('ROS loop is alive!')

    def send_pose_goal(self, pose: Pose):
        self.get_logger().info(f"Received move request: Position ({pose.position.x}, {pose.position.y}, {pose.position.z})")
        if self._move_in_progress:
            self.get_logger().warn("Move already in progress")
            return

        def _move_thread():
            try:
                self._move_in_progress = True
                self._move_done_event.clear()
                self.get_logger().info("Waiting for action server...")
                self._action_client.wait_for_server()

                goal_msg = MoveGroup.Goal()
                goal_msg.request.group_name = "ar_manipulator"
                goal_msg.request.num_planning_attempts = 5
                goal_msg.request.allowed_planning_time = 5.0
                goal_msg.request.max_velocity_scaling_factor = 1.0

                # Set the pose target directly
                goal_msg.request.goal_constraints = []
                position_constraint = Constraints()
                position_constraint.name = "pose_goal"

                # Create a position constraint
                pos_constraint = PositionConstraint()
                pos_constraint.header.frame_id = "base_link"
                pos_constraint.link_name = "ee_link"  # Your end effector link
                pos_constraint.target_point_offset = Vector3()
                pos_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
                primitive_pose = Pose()
                primitive_pose.position = pose.position
                pos_constraint.constraint_region.primitive_poses.append(primitive_pose)
                pos_constraint.weight = 1.0

                # Create an orientation constraint
                orient_constraint = OrientationConstraint()
                orient_constraint.header.frame_id = "base_link"
                orient_constraint.orientation = pose.orientation
                orient_constraint.link_name = "ee_link"  # Your end effector link
                orient_constraint.absolute_x_axis_tolerance = 0.1
                orient_constraint.absolute_y_axis_tolerance = 0.1
                orient_constraint.absolute_z_axis_tolerance = 0.1
                orient_constraint.weight = 1.0

                # Add constraints to the goal
                position_constraint.position_constraints.append(pos_constraint)
                position_constraint.orientation_constraints.append(orient_constraint)
                goal_msg.request.goal_constraints.append(position_constraint)

                # Workspace (optional)
                goal_msg.request.workspace_parameters = WorkspaceParameters()
                goal_msg.request.workspace_parameters.header.frame_id = "base_link"
                goal_msg.request.workspace_parameters.min_corner.x = -1.0
                goal_msg.request.workspace_parameters.min_corner.y = -1.0
                goal_msg.request.workspace_parameters.min_corner.z = -1.0
                goal_msg.request.workspace_parameters.max_corner.x = 1.0
                goal_msg.request.workspace_parameters.max_corner.y = 1.0
                goal_msg.request.workspace_parameters.max_corner.z = 1.0

                # Target pose
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "base_link"
                pose_stamped.pose = pose

                constraint = Constraints()
                constraint.name = "pose_goal"
                constraint.position_constraints = []  # You can fill this in if you want stricter constraints
                goal_msg.request.goal_constraints.append(constraint)

                # Send goal
                self.get_logger().info("Sending goal...")
                send_goal_future = self._action_client.send_goal_async(goal_msg)
                send_goal_future.add_done_callback(self._goal_response_callback)

            except Exception as e:
                self.get_logger().error(f"Exception: {e}")

        thread = threading.Thread(target=_move_thread, daemon=True)
        thread.start()

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by server.")
            self._move_in_progress = False
            self._move_done_event.set()
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_callback)
    
    def _get_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Action completed with result: {result.error_code.val}")
        except Exception as e:
            self.get_logger().error(f"Failed to get result: {e}")
        finally:
            self._move_in_progress = False
            self._move_done_event.set()

    def handle_move_ar(self, request: MoveToPose.Request, response:MoveToPose.Response):
        """Callback function for the 'ar_move_to' service."""
        self.get_logger().info(f"Received move request: Position ({request.pose.position.x}, {request.pose.position.y}, {request.pose.position.z})")
        self.get_logger().info(f"Orientation: ({request.pose.orientation.x}, {request.pose.orientation.y}, {request.pose.orientation.z}, {request.pose.orientation.w})")
        
        pose_to_move = request.pose
        
        pose_to_move = Pose()
        pose_to_move.position = Point(x=0.04, y=-0.31, z=0.375)

        pose_to_move.orientation.x = 0.044
        pose_to_move.orientation.y = -0.702
        pose_to_move.orientation.z = 0.71
        pose_to_move.orientation.w = -0.033

        

        #self.logger.info(f"moving to: {pose_to_move}")
        #time.sleep(2)
        #self.move_to(pose_to_move)
        self.send_pose_goal(pose_to_move)
        self._move_done_event.wait(30)  # Wait for 30 seconds or until the move is done
        response.success = True
        response.message = "Move command received"        
        return response  # ✅ This was missing

def main():

    # #Allow attaching the debugger remotely on port 5678
    # debugpy.listen(("0.0.0.0", 5678))
    # print("Waiting for debugger to attach...")
    # debugpy.wait_for_client()  # Uncomment this if you want to pause execution until the debugger attaches
    # print("debugger attached")
    
    # atexit.register(lambda: print("⚠️  Shutdown triggered via atexit"))
    # original_shutdown = rclpy.shutdown
    # def my_shutdown(*args, **kwargs):
    #     print("⚠️  rclpy.shutdown() was called!")
    #     return original_shutdown(*args, **kwargs)
    # rclpy.shutdown = my_shutdown
 

    rclpy.init()
    node = MoveAR()
    executor = MultiThreadedExecutor(30) 
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Exception: {e}")
    finally:
        node.get_logger().info("Shutting down...")
        rclpy.shutdown()


if __name__ == "__main__":
    main()

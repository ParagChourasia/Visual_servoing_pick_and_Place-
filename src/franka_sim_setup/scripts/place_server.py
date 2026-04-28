#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, Empty
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from franka_sim_interfaces.action import PlaceTask
import math

class PlaceActionServer(Node):

    def __init__(self):
        super().__init__('place_action_server',
                         parameter_overrides=[
                             rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)
                         ])
        
        self.callback_group = ReentrantCallbackGroup()
        
        # 1. Action Server
        self._action_server = ActionServer(
            self,
            PlaceTask,
            'place_task',
            execute_callback=self.execute_callback,
            callback_group=self.callback_group
        )
        
        # 2. Action Clients
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action', callback_group=self.callback_group)
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/franka_gripper_controller/follow_joint_trajectory', callback_group=self.callback_group)
        
        self.detach_pub = self.create_publisher(Empty, '/detachable_joint/detach', 10)
        
        self.current_joint_states = None
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Robot/Frame Setup
        self.base_frame = 'fer_link0'
        self.tcp_frame = 'fer_hand_tcp'
        self.group_name = 'fer_arm'

        # Dynamic Bin Subscriptions
        self.bin_locations = {
            "red":   None,
            "green": None,
            "blue":  None
        }
        self.create_subscription(Pose, '/bins/red',   lambda msg: self.bin_cb(msg, "red"),   10)
        self.create_subscription(Pose, '/bins/green', lambda msg: self.bin_cb(msg, "green"), 10)
        self.create_subscription(Pose, '/bins/blue',  lambda msg: self.bin_cb(msg, "blue"),  10)

        self.get_logger().info("PlaceTask Action Server Ready (with Dynamic Bin Tracking).")

    def joint_state_callback(self, msg): 
        self.current_joint_states = msg

    def bin_cb(self, msg, color):
        self.bin_locations[color] = msg

    # --- Main Action Logic ---
    async def execute_callback(self, goal_handle):
        target_color = goal_handle.request.color.lower()
        self.get_logger().info(f"Executing PlaceTask for color: {target_color}")
        
        feedback_msg = PlaceTask.Feedback()
        result = PlaceTask.Result()

        feedback_msg.status = f"Resolving {target_color} bin position..."
        goal_handle.publish_feedback(feedback_msg)
        
        dynamic_pose = self.bin_locations.get(target_color)
        if dynamic_pose is not None:
            self.get_logger().info(f"Captured dynamic bin location from overhead camera for {target_color}!")
            target_x = dynamic_pose.position.x
            target_y = dynamic_pose.position.y
        else:
            self.get_logger().warn(f"Dynamic bin {target_color} not found, using static fallback!")
            fallback_positions = {
                "red":   Point(x=0.8, y=0.4, z=0.2),
                "green": Point(x=0.8, y=0.0, z=0.2),
                "blue":  Point(x=0.8, y=-0.4, z=0.2)
            }
            fb = fallback_positions.get(target_color, Point(x=0.8, y=0.0, z=0.2))
            target_x = fb.x
            target_y = fb.y
            
        q_pick = [1.0, 0.0, 0.0, 0.0]
        
        # Reachability Safety Clamp (Max Reach ~0.82m)
        dist = math.hypot(target_x, target_y)
        if dist > 0.82:
            scale = 0.82 / dist
            target_x *= scale
            target_y *= scale
            self.get_logger().warn(f"Bin is outside reachable workspace (dist={dist:.2f}m). Clamping to 0.82m: X={target_x:.2f}, Y={target_y:.2f}")

        # Override Z height to 0.2m safe zone
        success = await self.move_to_pose(target_x, target_y, 0.2, q_pick)
        
        if not success:
            goal_handle.abort()
            result.success = False
            result.error_message = "Failed to reach bin position"
            return result
        
        feedback_msg.status = "Detaching object..."
        goal_handle.publish_feedback(feedback_msg)
        
        self.detach_pub.publish(Empty())
        
        # Open gripper
        await self.control_gripper(0.08)
        
        goal_handle.succeed()
        result.success = True
        return result

    # --- Utilities ---
    async def move_to_pose(self, x, y, z, q):
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.tcp_frame
        pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01]))
        pc.constraint_region.primitive_poses.append(Pose(position=Point(x=x, y=y, z=z)))
        c.position_constraints.append(pc)
        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.tcp_frame
        oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = [float(i) for i in q]
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = 0.1
        c.orientation_constraints.append(oc)
        goal.request.goal_constraints.append(c)
        return await self.send_action_goal(self.move_group_client, goal)

    async def control_gripper(self, width):
        if not self.current_joint_states: return False
        finger_joints = [name for name in self.current_joint_states.name if 'finger' in name.lower()]
        if not finger_joints: return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = finger_joints
        p = JointTrajectoryPoint(positions=[width/float(len(finger_joints))] * len(finger_joints))
        p.time_from_start.sec = 1
        goal.trajectory.points.append(p)
        return await self.send_action_goal(self.gripper_client, goal)

    async def send_action_goal(self, client, goal):
        if not client.wait_for_server(timeout_sec=2.0): return False
        goal_handle = await client.send_goal_async(goal)
        if not goal_handle.accepted: return False
        result = await goal_handle.get_result_async()
        return result.status == GoalStatus.STATUS_SUCCEEDED

def main(args=None):
    rclpy.init(args=args)
    node = PlaceActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

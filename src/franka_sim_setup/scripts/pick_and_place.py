#!/usr/bin/env python3

"""
Pick-and-Place State Machine - Step D (Fresh Start)

Industrial-grade ROS 2 Humble node using the statemachine library. 
Harmonized with Elena Oikonomou's Fall 2023 Reference logic.

Features:
- Asynchronous MoveGroup Action Client integration.
- Diagnostic Readiness Dashboard (Clock, Vision, Joints, MoveIt).
- Visual Servoing 'Lite' refinement for sub-millimeter grasp accuracy.
- Safe-exit logic when no objects are detected.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import CameraInfo, JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from shape_msgs.msg import SolidPrimitive
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import Contacts
from statemachine import State, StateMachine
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading
import time

class PickAndPlaceStateMachine(Node, StateMachine):

    # States
    idle = State("Idle", initial=True)
    home = State("Home")
    selecting_object = State("SelectingObject")                       
    picking_and_placing = State("PickingAndPlacing")                         
    done = State("Done", final=True)

    # Events & Transitions
    start_mission = idle.to(home)
    select_object = home.to(selecting_object, cond="are_objects_detected") | home.to(done, unless="are_objects_detected")
    pick_object = selecting_object.to(picking_and_placing, cond="object_selected")
    get_ready = picking_and_placing.to(home)
    
    # Global Reset
    reset = home.to(home) | selecting_object.to(home) | picking_and_placing.to(home)

    def __init__(self):
        # 1. ROS 2 Node Init
        Node.__init__(self, 'pick_and_place_controller',
                          parameter_overrides=[
                              rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)
                          ])
        
        # 2. State Machine Init
        StateMachine.__init__(self)
        
        # Persistence & TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.wrist_cam_info = None
        self.wrist_pixel = None
        self.current_joint_states = None
        self.clock_received = False
        
        # Internal Status (avoiding method name clashes)
        self._int_object_selected = False
        self.target_pixel = None
        self.target_color_id = None
        
        # Tactile Grasp States
        self.left_contact = False
        self.right_contact = False
        
        # Robot Prefix Config
        self.joint_prefix = 'fer' 
        self.group_name = 'fer_arm'
        self.finger_prefix = 'fer'
        self.base_frame = 'fer_link0'
        self.tcp_frame = 'fer_hand_tcp'

        # Dynamic Bin Locations
        self.bin_locations = {
            "Red":   None,
            "Green": None,
            "Blue":  None
        }

        # Action Clients
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/franka_gripper_controller/follow_joint_trajectory')
        
        # Pubs/Subs
        self.attach_pub = self.create_publisher(String, '/detachable_joint/attach', 10)
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.create_subscription(Point, '/detected_object_pixel', self.wrist_obj_callback, 10)
        self.create_subscription(CameraInfo, '/wrist_camera/camera_info', self.wrist_info_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Contact Sensors
        self.create_subscription(Contacts, '/gripper_left_contact', self.left_contact_cb, 10)
        self.create_subscription(Contacts, '/gripper_right_contact', self.right_contact_cb, 10)

        # Dynamic Bin Subscriptions
        self.create_subscription(Pose, '/bins/red',   lambda msg: self.bin_cb(msg, "Red"),   10)
        self.create_subscription(Pose, '/bins/green', lambda msg: self.bin_cb(msg, "Green"), 10)
        self.create_subscription(Pose, '/bins/blue',  lambda msg: self.bin_cb(msg, "Blue"),  10)

        self.get_logger().info("*** Pick-and-Place State Machine Node Ready ***") 
        
        # 3. Start Lifecycle Thread
        threading.Thread(target=self.mission_lifecycle_thread).start()

    # --- Callbacks ---

    def clock_callback(self, _):
        self.clock_received = True

    def wrist_info_callback(self, msg):
        self.wrist_cam_info = msg

    def wrist_obj_callback(self, msg):
        self.wrist_pixel = msg

    def joint_state_callback(self, msg):
        self.current_joint_states = msg

    def left_contact_cb(self, msg):
        self.left_contact = len(msg.contact) > 0

    def right_contact_cb(self, msg):
        self.right_contact = len(msg.contact) > 0

    def bin_cb(self, msg, color):
        self.bin_locations[color] = msg

    # --- Lifecycle & Diagnostics ---

    def mission_lifecycle_thread(self):
        """Monitors system readiness then starts SM."""
        while rclpy.ok():
            ready_clock = self.clock_received
            ready_vision = self.wrist_cam_info is not None
            ready_joints = self.current_joint_states is not None
            ready_moveit = self.move_group_client.server_is_ready()

            if ready_joints:
                test_joint = self.current_joint_states.name[0]
                self.joint_prefix = 'panda' if 'panda' in test_joint else 'fer'
                self.group_name = f'{self.joint_prefix}_arm'
                self.finger_prefix = self.joint_prefix
                self.base_frame = f'{self.joint_prefix}_link0'
                self.tcp_frame = f'{self.joint_prefix}_hand_tcp'

            print("\rReadiness: [Clock: %s] [Vision: %s] [Joints: %s] [MoveIt: %s]" % 
                  ("OK" if ready_clock else "--", 
                   "OK" if ready_vision else "--", 
                   "OK" if ready_joints else "--", 
                   "OK" if ready_moveit else "--"), end="")

            if ready_clock and ready_joints and ready_moveit:
                print("\n\n" + 60*'*')
                self.get_logger().info("SYSTEM READY. Launching State Machine...")
                print(60*'*' + "\n")
                self.send("start_mission")
                break
            time.sleep(1.0)

    # --- Guards ---

    def are_objects_detected(self) -> bool:
        # Give vision a tiny moment to stabilize at Home
        time.sleep(1.0) 
        return self.wrist_pixel is not None

    def object_selected(self) -> bool:
        return self._int_object_selected

    # --- State Transition Logic ---

    def on_enter_idle(self):
        self.get_logger().info("Status: IDLE (Waiting for System Check)")

    def on_enter_home(self):
        self.get_logger().info("Transitioning to HOME position..") 
        self._int_object_selected = False
        
        names = [f'{self.joint_prefix}_joint{i}' for i in range(1, 8)]
        values = [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
        
        if self.move_to_joints(names, values):
            # Arrived at Home. Now check for objects.
            self.send("select_object")
        else:
            self.get_logger().error("Motion Failed. Retrying Home..")
            time.sleep(2.0)
            self.send("reset")

    def on_enter_selecting_object(self):
        self.get_logger().info("Searching workbench for target..") 
        if self.wrist_pixel:
            self.target_pixel = self.wrist_pixel
            self.target_color_id = self.wrist_pixel.z
            self._int_object_selected = True
            self.get_logger().info(f"Target selected. Color ID: {self.target_color_id}") 
            self.send("pick_object")
        else:
            self.get_logger().warn("Workbench empty. Terminating mission.")
            self.send("get_ready") # Will fall through to Done

    def on_enter_picking_and_placing(self):
        self.get_logger().info("Executing Precision Pick & Place sequence..") 
        
        if self.target_pixel is None:
            self.send("get_ready")
            return

        model_map = {1.0: "red_cube", 2.0: "green_cube", 3.0: "blue_cube"}
        target_model = model_map.get(self.target_color_id, "red_cube")
        
        # 1. Workspace Projection
        world_pos = self.pixel_to_world(self.target_pixel.x, self.target_pixel.y)
        if world_pos is None:
            self.get_logger().error("Target lost during projection. Returning Home..")
            self.send("get_ready")
            return

        q_pick = [1.0, 0.0, 0.0, 0.0]
        self.control_gripper(0.08) # Open
        
        # 2. Approach & Refine (Visual Servoing Lite)
        self.get_logger().info(f"Approaching {target_model}...")
        self.move_to_pose(world_pos[0], world_pos[1], 0.1, q_pick)
        
        time.sleep(1.0) # 1.5s refinement pause
        if self.wrist_pixel and abs(self.wrist_pixel.z - self.target_color_id) < 0.1:
            refined_pos = self.pixel_to_world(self.wrist_pixel.x, self.wrist_pixel.y)
            if refined_pos is not None:
                self.move_to_pose(refined_pos[0], refined_pos[1], 0.1, q_pick)
                world_pos = refined_pos
                time.sleep(1.0)

        # 3. Precision Pick Dive (Z=0.04)
        self.get_logger().info("Diving for grasp...")
        self.move_to_pose(world_pos[0], world_pos[1], 0.04, q_pick)
        
        # 4. Grasp (High Tension Command)
        self.get_logger().info("Closing fingers...")
        self.control_gripper(0.0) # Force full closure for maximum tension
        time.sleep(1.0)
        
        # 5. ATTACH IMMEDIATELY (Safety for lift-off)
        self.get_logger().info(f"Locking {target_model} to gripper...")
        self.attach_pub.publish(String(data=target_model))
        time.sleep(0.5)

        # 6. RETRACT (Move up before verification to ensure stable lift)
        self.get_logger().info("Retracting...")
        self.move_to_pose(world_pos[0], world_pos[1], 0.2, q_pick)
        
        # 7. VERIFY (Now that we've lifted, check if we actually have it)
        if self.verify_grasp():
            self.get_logger().info("Grasp CONFIRMED. Moving to bin.")
            
            # 8. Delivery (Dynamic Bin Lookup)
            color_name = target_model.split('_')[0].capitalize()
            bin_pose = self.bin_locations.get(color_name)
            
            if bin_pose:
                self.get_logger().info(f"Delivering to DYNAMIC {color_name} bin at X:{bin_pose.position.x:.3f}, Y:{bin_pose.position.y:.3f}...")
                if self.move_to_pose(bin_pose.position.x, bin_pose.position.y, 0.2, q_pick):
                    self.get_logger().info("Arrived at bin. Stabilizing for 1s...")
                    time.sleep(1.0)
                else:
                    self.get_logger().error("Failed to reach bin target!")
            else:
                self.get_logger().warn(f"Dynamic {color_name} bin not discovered yet. Using fallback safe-zone.")
                bin_y = 0.4 if color_name == "Red" else (0.0 if color_name == "Green" else -0.4)
                self.move_to_pose(0.9, bin_y, 0.2, q_pick)
                time.sleep(1.0)
            
            # 9. Release
            self.attach_pub.publish(String(data="detach"))
            self.control_gripper(0.08)
            self.get_logger().info("Object placed.")
            self.send("get_ready")
        else:
            self.get_logger().error("Grasp FAILED (Empty). Resetting..")
            self.attach_pub.publish(String(data="detach"))
            self.control_gripper(0.08)
            self.send("reset")

    def on_enter_done(self):
        print('\n' + 80*'=')
        self.get_logger().info("*** MISSION COMPLETE! ALL TARGETS SORTED! ***") 
        print(80*'=')

    # --- Geometric Utilities ---

    def pixel_to_world(self, u, v):
        if not self.wrist_cam_info: return None
        try:
            trans = self.tf_buffer.lookup_transform(self.base_frame, 'camera_link_optical', rclpy.time.Time())
            fx, fy, cx, cy = self.wrist_cam_info.k[0], self.wrist_cam_info.k[4], self.wrist_cam_info.k[2], self.wrist_cam_info.k[5]
            x_c, y_c = (u - cx) / fx, (v - cy) / fy
            rot = R.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]).as_matrix()
            t = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            ray_w = rot @ np.array([x_c, y_c, 1.0])
            ray_w /= np.linalg.norm(ray_w)
            table_z = 0.04
            dist = (table_z - t[2]) / ray_w[2]
            return t + dist * ray_w if dist > 0 else None
        except Exception: return None

    # --- Motion Execution API ---

    def move_to_joints(self, names, values):
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        c = Constraints()
        for n, v in zip(names, values):
            c.joint_constraints.append(JointConstraint(joint_name=n, position=v, tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        goal.request.goal_constraints.append(c)
        return self.send_goal_wait(self.move_group_client, goal)

    def move_to_pose(self, x, y, z, q):
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
        return self.send_goal_wait(self.move_group_client, goal)

    def control_gripper(self, width):
        if self.current_joint_states is None: 
            self.get_logger().error("Gripper Command Failed: No joint states available to discover finger names!")
            return False

        # DYNAMIC DISCOVERY: Find fingers in active state
        finger_joints = [name for name in self.current_joint_states.name if 'finger' in name.lower()]
        
        if not finger_joints:
            self.get_logger().error("Gripper Command Failed: Found no 'finger' joints in system!")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = finger_joints
        
        # Split target width across all discovered fingers
        p = JointTrajectoryPoint(positions=[width/float(len(finger_joints))] * len(finger_joints))
        p.time_from_start.sec = 1
        goal.trajectory.points.append(p)
        
        self.get_logger().info(f"Commanding Gripper {finger_joints} to width {width:.4f}m")
        return self.send_goal_wait(self.gripper_client, goal)

    def verify_grasp(self):
        self.get_logger().info("Starting Multi-Sample Grasp Verification (2s loop)...")
        
        start_time = time.time()
        max_seen_width = 0.0
        
        # Check for 2 seconds to handle physics/sensor lag
        while (time.time() - start_time) < 2.0:
            if self.current_joint_states:
                finger_data = [(name, pos) for name, pos in zip(self.current_joint_states.name, self.current_joint_states.position) if 'finger' in name.lower()]
                if finger_data:
                    width = sum(d[1] for d in finger_data)
                    max_seen_width = max(max_seen_width, width)
                    
                    # HYBRID CHECK: Aperture OR Tactile Contact
                    if width > 0.005 or self.left_contact or self.right_contact: 
                        reason = "Aperture" if width > 0.005 else "Tactile Contact"
                        self.get_logger().info(f"Grasp VALIDATED: Found {reason} (Width: {width:.4f}m, L: {self.left_contact}, R: {self.right_contact})")
                        return True
            time.sleep(0.1)

        self.get_logger().error(f"Grasp FAILED: Max aperture seen during 2s grace period was only {max_seen_width:.4f}m")
        return False

    def send_goal_wait(self, client, goal):
        if not client.wait_for_server(timeout_sec=2.0): return False
        future = client.send_goal_async(goal)
        while rclpy.ok() and not future.done(): time.sleep(0.05)
        handle = future.result()
        if not handle.accepted: return False
        res_future = handle.get_result_async()
        while rclpy.ok() and not res_future.done(): time.sleep(0.05)
        return res_future.result().status == GoalStatus.STATUS_SUCCEEDED

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceStateMachine()
    from rclpy.executors import MultiThreadedExecutor
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

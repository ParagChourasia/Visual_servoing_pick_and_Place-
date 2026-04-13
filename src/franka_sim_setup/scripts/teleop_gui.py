#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time

from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

class FrankaTeleopGUI(Node):
    def __init__(self):
        super().__init__('franka_teleop_gui')

        # 1. Action Clients
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/franka_gripper_controller/follow_joint_trajectory')
        self.attach_pub = self.create_publisher(String, '/detachable_joint/attach', 10)
        
        # 2. State & Config
        self.current_joint_states = None
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        
        self.joint_prefix = 'fer'
        self.group_name = 'fer_arm'
        self.tcp_frame = 'fer_hand_tcp'
        self.base_frame = 'fer_link0'

        # 3. Dynamic Bin Discovery
        self.bin_locations = {
            "Red":   None,
            "Green": None,
            "Blue":  None
        }
        self.create_subscription(Pose, '/bins/red',   lambda msg: self.bin_cb(msg, "Red"),   10)
        self.create_subscription(Pose, '/bins/green', lambda msg: self.bin_cb(msg, "Green"), 10)
        self.create_subscription(Pose, '/bins/blue',  lambda msg: self.bin_cb(msg, "Blue"),  10)

        self.get_logger().info("Franka Teleop GUI Node Ready.")

    def joint_state_cb(self, msg):
        self.current_joint_states = msg
        # Detect prefix dynamically (always update to stay current)
        if len(msg.name) > 0:
            test_joint = msg.name[0]
            new_prefix = 'panda' if 'panda' in test_joint else 'fer'
            if new_prefix != self.joint_prefix:
                self.joint_prefix = new_prefix
                self.group_name = f'{self.joint_prefix}_arm'
                self.tcp_frame = f'{self.joint_prefix}_hand_tcp'
                self.base_frame = f'{self.joint_prefix}_link0'
                self.get_logger().info(f"System Detected: {self.joint_prefix} robot configuration.")

    def bin_cb(self, msg, color):
        self.bin_locations[color] = msg

    # --- Motion Methods ---

    def move_to_pose(self, x, y, z, q=[1.0, 0.0, 0.0, 0.0]):
        # Reachability Guard (Franka Reach ~0.85m)
        if x > 0.85:
            self.get_logger().warn(f"Target X={x} exceeds safe reach. Capping to 0.82m.")
            x = 0.82
        
        # Ensure q is a list of floats
        q = [float(val) for val in q]

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        
        c = Constraints()
        # Position
        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.tcp_frame
        pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.02, 0.02, 0.02])) # Relaxed to 2cm
        pc.constraint_region.primitive_poses.append(Pose(position=Point(x=x, y=y, z=z)))
        pc.weight = 1.0
        c.position_constraints.append(pc)
        
        goal.request.goal_constraints.append(c)
        
        # Strip back to absolute essentials matching pick_and_place.py
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        
        self.get_logger().info(f"Sending minimal goal to {self.group_name}: X={x}, Y={y}, Z={z}")
        self.send_goal_async(self.move_group_client, goal)

    def control_gripper(self, width):
        if self.current_joint_states is None: 
            return
        
        finger_joints = [name for name in self.current_joint_states.name if 'finger' in name.lower()]
        if not finger_joints: return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = finger_joints
        p = JointTrajectoryPoint(positions=[width/2.0] * len(finger_joints))
        p.time_from_start.sec = 1
        goal.trajectory.points.append(p)
        
        self.get_logger().info(f"Gripper to {width}m")
        self.send_goal_async(self.gripper_client, goal)

    def send_goal_async(self, client, goal):
        self.get_logger().info(f"Waiting for {client._action_name} server...")
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Action server {client._action_name} TIMEOUT!")
            return
        
        self.get_logger().info(f"Sending async goal to {client._action_name}...")
        future = client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("GOAL REJECTED by server!")
            return

        self.get_logger().info("Goal accepted. Executing...")
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("MOTION SUCCESSFUL! Target reached.")
        else:
            self.get_logger().error(f"MOTION FAILED! Status code: {status}")

# --- GUI Class ---

class TeleopDashboard:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Franka Panda - Interactive Teleop")
        self.root.geometry("400x600")
        self.root.configure(bg="#1e1e2e")

        style = ttk.Style()
        style.theme_use('clam')
        style.configure("TFrame", background="#1e1e2e")
        style.configure("TLabel", background="#1e1e2e", foreground="#cdd6f4", font=("Arial", 10))
        style.configure("TButton", padding=5, font=("Arial", 10, "bold"))

        # 1. Header
        header = ttk.Label(self.root, text="ROBOT TELEOP CONTROL", font=("Arial", 16, "bold"), foreground="#fab387")
        header.pack(pady=20)

        # 2. Cartesian Input
        pose_frame = ttk.LabelFrame(self.root, text=" Cartesian Target (Meters) ", padding=10)
        pose_frame.pack(padx=20, fill="x")

        ttk.Label(pose_frame, text="X:").grid(row=0, column=0, pady=5)
        self.ent_x = ttk.Entry(pose_frame, width=10)
        self.ent_x.insert(0, "0.5")
        self.ent_x.grid(row=0, column=1)

        ttk.Label(pose_frame, text="Y:").grid(row=1, column=0, pady=5)
        self.ent_y = ttk.Entry(pose_frame, width=10)
        self.ent_y.insert(0, "0.0")
        self.ent_y.grid(row=1, column=1)

        ttk.Label(pose_frame, text="Z:").grid(row=2, column=0, pady=5)
        self.ent_z = ttk.Entry(pose_frame, width=10)
        self.ent_z.insert(0, "0.2")
        self.ent_z.grid(row=2, column=1)

        btn_move = tk.Button(pose_frame, text="EXECUTE MOVE", command=self.do_move, bg="#a6e3a1", fg="#11111b", font=("Arial", 10, "bold"))
        btn_move.grid(row=3, column=0, columnspan=2, pady=10, sticky="ew")

        # 3. Gripper Control
        grip_frame = ttk.LabelFrame(self.root, text=" End Effector Control ", padding=10)
        grip_frame.pack(padx=20, pady=10, fill="x")

        btn_open = tk.Button(grip_frame, text="OPEN GRIPPER", command=lambda: self.node.control_gripper(0.08), bg="#89dceb", width=15)
        btn_open.pack(side="left", padx=5, expand=True)

        btn_close = tk.Button(grip_frame, text="CLOSE GRIPPER", command=lambda: self.node.control_gripper(0.0), bg="#f38ba8", width=15)
        btn_close.pack(side="right", padx=5, expand=True)

        # 4. Presets
        preset_frame = ttk.LabelFrame(self.root, text=" Fast Presets ", padding=10)
        preset_frame.pack(padx=20, pady=10, fill="x")

        presets = [
            ("HOME", 0.3, 0.0, 0.5, "Home"),
            ("RED BIN", 0.8, 0.4, 0.2, "Red"),
            ("GREEN BIN", 0.8, 0.0, 0.2, "Green"),
            ("BLUE BIN", 0.8, -0.4, 0.2, "Blue")
        ]

        for text, px, py, pz, color_key in presets:
            btn = ttk.Button(preset_frame, text=text, command=lambda x=px, y=py, z=pz, c=color_key: self.do_preset_move(x, y, z, c))
            btn.pack(fill="x", pady=2)

        # 5. Safety/Etc
        etc_frame = ttk.LabelFrame(self.root, text=" System Overrides ", padding=10)
        etc_frame.pack(padx=20, pady=10, fill="x")

        btn_detach = tk.Button(etc_frame, text="EMERGENCY DETACH", command=lambda: self.node.attach_pub.publish(String(data="detach")), bg="#fab387", fg="#11111b")
        btn_detach.pack(fill="x")

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def do_move(self):
        try:
            x = float(self.ent_x.get())
            y = float(self.ent_y.get())
            z = float(self.ent_z.get())
            self.node.move_to_pose(x, y, z)
        except ValueError:
            messagebox.showerror("Input Error", "Please enter valid numerical coordinates.")

    def do_preset_move(self, fallback_x, fallback_y, fallback_z, color):
        q_pick = [1.0, 0.0, 0.0, 0.0]
        dynamic_pose = self.node.bin_locations.get(color)
        if dynamic_pose:
            self.node.get_logger().info(f"MOVING TO DYNAMIC {color} BIN FROM VISION")
            self.node.move_to_pose(dynamic_pose.position.x, dynamic_pose.position.y, 0.2, q_pick)
        else:
            self.node.get_logger().info(f"MOVING TO STATIC {color} PRESET (FALLBACK)")
            self.node.move_to_pose(fallback_x, fallback_y, fallback_z, q_pick)

    def on_close(self):
        self.root.destroy()
        rclpy.shutdown()

    def run(self):
        self.root.mainloop()

def main():
    rclpy.init()
    node = FrankaTeleopGUI()
    
    # Run ROS spin in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Main thread runs the GUI
    ui = TeleopDashboard(node)
    ui.run()

if __name__ == '__main__':
    main()

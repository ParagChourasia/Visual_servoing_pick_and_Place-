#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ros_gz_interfaces.msg import Contacts
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import os
import time
import math

class GripperMonitor(Node):
    def __init__(self):
        super().__init__('gripper_monitor')
        
        # TF Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Joint State
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.latest_joints = None
        
        # Tactile Sensing (FSR simulation)
        self.create_subscription(Contacts, '/gripper_left_contact', self.left_contact_cb, 10)
        self.create_subscription(Contacts, '/gripper_right_contact', self.right_contact_cb, 10)
        self.left_fsr = 0.0
        self.right_fsr = 0.0
        
        # Prefix Discovery
        self.base_frame = 'fer_link0'
        self.tcp_frame = 'fer_hand_tcp'
        self.prefix_detected = False

        self.get_logger().info('Gripper Telemetry Monitor Started.')
        
        # Timer for Dashboard Update (10Hz)
        self.create_timer(0.1, self.dashboard_timer)

    def joint_callback(self, msg):
        self.latest_joints = msg
        if not self.prefix_detected and len(msg.name) > 0:
            pref = 'panda' if 'panda' in msg.name[0] else 'fer'
            self.base_frame = f'{pref}_link0'
            self.tcp_frame = f'{pref}_hand_tcp'
            self.prefix_detected = True

    def left_contact_cb(self, msg):
        self.left_fsr = self.calculate_force(msg)

    def right_contact_cb(self, msg):
        self.right_fsr = self.calculate_force(msg)

    def calculate_force(self, msg):
        total_f = 0.0
        for c in msg.contact:
            for w in c.wrench:
                f = w.force
                total_f += math.sqrt(f.x**2 + f.y**2 + f.z**2)
        return total_f

    def get_gripper_state(self, width):
        if width > 0.07: return "OPEN"
        if width > 0.01: return "GRASPED / CLOSED ON OBJECT"
        return "FULLY CLOSED"

    def dashboard_timer(self):
        # 1. Get Pose
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.base_frame, self.tcp_frame, now)
            x, y, z = trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z
            rx, ry, rz, rw = trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w
        except (LookupException, ConnectivityException, ExtrapolationException):
            x = y = z = 0.0
            rx = ry = rz = 0.0
            rw = 1.0

        # 2. Get Width
        width = 0.0
        state = "UNKNOWN"
        if self.latest_joints:
            finger_indices = [i for i, name in enumerate(self.latest_joints.name) if 'finger' in name.lower()]
            width = sum(self.latest_joints.position[i] for i in finger_indices)
            state = self.get_gripper_state(width)

        # 3. Clear Screen & Print Dashboard
        os.system('clear')
        print("="*60)
        print("         FRANKA PANDA LIVE TELEMETRY DASHBOARD")
        print("="*60)
        print(f" END EFFECTOR POSE (Frame: {self.base_frame} -> {self.tcp_frame})")
        print(f"   X: {x:8.4f} m")
        print(f"   Y: {y:8.4f} m")
        print(f"   Z: {z:8.4f} m")
        print("-" * 60)
        print(f" GRIPPER STATUS")
        print(f"   Aperture (Width): {width:8.4f} m")
        print(f"   Tactile Force (L): {self.left_fsr:8.2f} N")
        print(f"   Tactile Force (R): {self.right_fsr:8.2f} N")
        
        # Determine State
        if "GRASPED" in state or (self.left_fsr > 0.1 and self.right_fsr > 0.1):
             state = "\033[1;32mGRASPED (Tactile Confirmation)\033[0m"
             
        print(f"   Current State:    {state}")
        print("="*60)
        print(" [Press Ctrl+C to Exit]")

def main(args=None):
    rclpy.init(args=args)
    monitor = GripperMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

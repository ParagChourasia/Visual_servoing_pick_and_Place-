#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from tf2_ros import Buffer, TransformListener

class OverheadDetector(Node):
    def __init__(self):
        super().__init__('overhead_detector')
        
        # ROS 2 Setup
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_depth = None
        self.cam_info = None
        
        # Configuration
        self.camera_topic = '/overhead_camera/image'
        self.depth_topic = '/overhead_camera/depth_image'
        self.info_topic = '/overhead_camera/camera_info'
        
        # Camera Pose (Fixed in World)
        # Pose: [0.8 0 1.2] rpy [0 1.5708 0]
        self.cam_x, self.cam_y, self.cam_z = 0.8, 0.0, 1.2
        
        # Subscriptions
        self.create_subscription(Image, self.camera_topic, self.image_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(CameraInfo, self.info_topic, self.info_cb, 10)
        
        # Publishers
        self.cube_pub = self.create_publisher(PoseArray, '/global_cubes', 10)
        self.bin_pubs = {
            "Red": self.create_publisher(Pose, '/bins/red', 10),
            "Green": self.create_publisher(Pose, '/bins/green', 10),
            "Blue": self.create_publisher(Pose, '/bins/blue', 10)
        }
        self.marker_pub = self.create_publisher(MarkerArray, '/overhead_detector/markers', 10)
        self.debug_pub = self.create_publisher(Image, '/overhead_detector/debug_image', 10)
        
        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer for detection (10Hz)
        self.create_timer(0.1, self.detect_objects)
        
        self.get_logger().info("Overhead Detector Node Started.")

    def image_cb(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def info_cb(self, msg):
        self.cam_info = msg

    def pixel_to_world(self, u, v, depth):
        if self.cam_info is None: return None
        
        # 1. Camera Intrinsics
        fx = self.cam_info.k[0]
        fy = self.cam_info.k[4]
        cx = self.cam_info.k[2]
        cy = self.cam_info.k[5]
        
        # 2. Camera Coordinates (Optical: Z-forward, X-right, Y-down)
        z_c = float(depth)
        x_c = (u - cx) * z_c / fx
        y_c = (v - cy) * z_c / fy

        # 3. Transform to Robot Base (fer_link0)
        try:
            # We assume a fixed frame exists for the camera or we use the known transform.
            # In simulation, the link is overhead_camera_link.
            # However, since it's a model in the SDF, we might not have a TF broadaster for it yet.
            # We'll use the hardcoded transform but FIX THE MATH based on Pose [0.8 0 1.2] rpy [0 1.5708 0]
            
            # Orientation: Pitched 90 deg down.
            # P_world = P_cam_origin + R_y(90) * P_optical
            # R_y(90) = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
            # P_world_x = Cam_x + P_optical_z
            # P_world_y = Cam_y + P_optical_x
            # P_world_z = Cam_z - P_optical_y (optical Y is down, so positive Y is lower world Z)
            # Wait, if CX/CY is center, and X is right (World Y)...
            
            world_x = self.cam_x + 0.0 # Optical Z doesn't change World X much if looking straight down? 
            # Actually, if looking down, Optical Z is World -Z.
            # Let's re-calculate:
            # SDF Pose: 0.8 0 1.2 rpy: 0 1.57 0
            # Pitch 90 around Y.
            # World X -> Cam -Z
            # World Y -> Cam X
            # World Z -> Cam -Y 
            
            # Correction:
            world_x = self.cam_x - y_c
            world_y = self.cam_y + x_c
            world_z = self.cam_z - z_c
            
            return (world_x, world_y, world_z)
        except Exception as e:
            self.get_logger().error(f"Transform failed: {e}")
            return None

    def detect_objects(self):
        if self.latest_image is None or self.latest_depth is None or self.cam_info is None:
            return

        # Color Masks
        hsv = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)
        colors = {
            "Red": ([0, 150, 50], [10, 255, 255]),
            "Green": ([35, 100, 50], [85, 255, 255]),
            "Blue": ([100, 100, 50], [165, 255, 255])
        }

        cube_poses = PoseArray()
        cube_poses.header.frame_id = "fer_link0"
        cube_poses.header.stamp = self.get_clock().now().to_msg()
        
        marker_array = MarkerArray()
        debug_img = self.latest_image.copy()

        obj_id = 0
        for name, (low, high) in colors.items():
            mask = cv2.inRange(hsv, np.array(low), np.array(high))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 200: continue
                
                is_bin = area > 1500 # Bin vs Cube threshold
                
                # Centroid
                M = cv2.moments(cnt)
                if M['m00'] == 0: continue
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # Depth
                depth = self.latest_depth[cy, cx]
                if np.isnan(depth) or depth <= 0: continue
                
                # 3D Position
                pos = self.pixel_to_world(cx, cy, depth)
                if pos:
                    wx, wy, wz = pos
                    
                    p = Pose()
                    p.position.x = wx
                    p.position.y = wy
                    p.position.z = wz
                    
                    if is_bin:
                        self.bin_pubs[name].publish(p)
                    else:
                        cube_poses.poses.append(p)

                    # Marker
                    marker = Marker()
                    marker.header.frame_id = "fer_link0"
                    marker.ns = f"{name}_{'BIN' if is_bin else 'CUBE'}"
                    marker.id = obj_id
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose = p
                    marker.scale.x = marker.scale.y = 0.2 if is_bin else 0.04
                    marker.scale.z = 0.02 if is_bin else 0.04
                    marker.color.a = 0.5 if is_bin else 0.9
                    if name == "Red": marker.color.r = 1.0
                    elif name == "Green": marker.color.g = 1.0
                    elif name == "Blue": marker.color.b = 1.0
                    marker_array.markers.append(marker)
                    
                    # HUD HUD (Premium Aesthetics)
                    color_bgr = (0,0,255) if name=="Red" else (0,255,0) if name=="Green" else (255,0,0)
                    cv2.drawContours(debug_img, [cnt], -1, color_bgr, 2 if not is_bin else 4)
                    label = f"{'BIN' if is_bin else 'CUBE'}: {name}"
                    cv2.putText(debug_img, label, (cx-30, cy-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                    obj_id += 1

        # GLOBAL HUD
        cv2.putText(debug_img, "OVERHEAD GLOBAL VISION (10Hz)", (20, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(debug_img, f"Active Objects: {obj_id}", (20, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        cv2.imshow("Overhead Global Perception", debug_img)
        cv2.waitKey(1)

        self.cube_pub.publish(cube_poses)
        self.marker_pub.publish(marker_array)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))

def main():
    rclpy.init()
    node = OverheadDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

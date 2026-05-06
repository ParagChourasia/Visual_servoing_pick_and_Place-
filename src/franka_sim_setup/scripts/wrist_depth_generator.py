#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class WristDepthGenerator(Node):
    def __init__(self):
        super().__init__('wrist_depth_generator')
        
        # ROS 2 Setup
        self.bridge = CvBridge()
        self.depth_topic = '/wrist_camera/depth_image'
        
        # Subscriptions
        self.subscription = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )
        
        # Performance monitoring
        self.last_time = time.time()
        self.fps = 0
        self.frame_count = 0
        
        # Gripper Offset (Approx distance from camera to fingertips in meters)
        # Based on URDF: Camera is at z=0.04 in fer_hand, Fingertips are at ~z=0.103
        self.GRIPPER_FINGERTIP_OFFSET = 0.063 
        
        self.get_logger().info(f"Wrist Depth Generator started. Subscribed to {self.depth_topic}")

    def depth_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            if depth_image is None:
                return

            depth_array = np.array(depth_image, dtype=np.float32)
            
            # Filter out NaN and Inf values
            depth_array = np.nan_to_num(depth_array, nan=0.0, posinf=10.0, neginf=0.0)
            
            # Calculate statistics for the HUD
            valid_depths = depth_array[depth_array > 0.05] # Ignore very close noise
            min_depth = np.min(valid_depths) if len(valid_depths) > 0 else 0.0
            max_depth = np.max(valid_depths) if len(valid_depths) > 0 else 0.0
            mean_depth = np.mean(valid_depths) if len(valid_depths) > 0 else 0.0
            
            # Normalize for visualization (0 to 255)
            # Focus range: 0.05m to 1.0m for wrist camera
            norm_depth = cv2.normalize(np.clip(depth_array, 0.05, 1.0), None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # Apply Color Map (MAGMA for a distinct "Wrist" look)
            depth_colormap = cv2.applyColorMap(norm_depth, cv2.COLORMAP_MAGMA)
            
            # Find nearest object (excluding gripper itself if visible)
            # Thresholding for objects within 0.5m of the camera
            object_mask = (depth_array > self.GRIPPER_FINGERTIP_OFFSET) & (depth_array < 0.6)
            object_mask = object_mask.astype(np.uint8) * 255
            
            kernel = np.ones((5,5), np.uint8)
            object_mask = cv2.morphologyEx(object_mask, cv2.MORPH_OPEN, kernel)
            
            contours, _ = cv2.findContours(object_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            num_objs = 0
            for cnt in contours:
                if cv2.contourArea(cnt) < 400: continue
                
                # Get average depth in this contour
                mask = np.zeros(depth_array.shape, np.uint8)
                cv2.drawContours(mask, [cnt], -1, 255, -1)
                avg_depth = cv2.mean(depth_array, mask=mask)[0]
                
                # Distance from Fingertips
                dist_from_fingertips = avg_depth - self.GRIPPER_FINGERTIP_OFFSET
                
                # Centroid
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    # Draw visual markers
                    cv2.drawContours(depth_colormap, [cnt], -1, (0, 255, 0), 2)
                    label = f"Dist: {dist_from_fingertips*100:.1f}cm"
                    cv2.putText(depth_colormap, label, (cx - 40, cy), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                    num_objs += 1

            # HUD
            self.draw_hud(depth_colormap, min_depth, max_depth, mean_depth, num_objs)
            
            cv2.imshow("Wrist Camera - Live Depth Map", depth_colormap)
            cv2.waitKey(1)
            
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                curr_time = time.time()
                self.fps = 30 / (curr_time - self.last_time)
                self.last_time = curr_time

        except Exception as e:
            self.get_logger().error(f"Error in depth_callback: {e}")

    def draw_hud(self, img, min_d, max_d, mean_d, num_objs):
        h, w, _ = img.shape
        
        # Header
        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (w, 60), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, img, 0.3, 0, img)
        
        cv2.putText(img, "WRIST DEPTH ANALYZER", (20, 35), 
                    cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255, 255), 2)
        
        cv2.putText(img, f"FPS: {self.fps:.1f} | Objects: {num_objs}", (w - 250, 35), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        
        # Bottom HUD
        cv2.rectangle(img, (0, h - 40), (w, h), (20, 20, 20), -1)
        cv2.putText(img, f"Near: {min_d:.2f}m | Far: {max_d:.2f}m | Fingertip Offset: {self.GRIPPER_FINGERTIP_OFFSET*100:.1f}cm", 
                    (20, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

def main(args=None):
    rclpy.init(args=args)
    node = WristDepthGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

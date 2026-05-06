#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class OverheadDepthGenerator(Node):
    def __init__(self):
        super().__init__('overhead_depth_generator')
        
        # ROS 2 Setup
        self.bridge = CvBridge()
        self.depth_topic = '/overhead_camera/depth_image'
        
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
        
        self.get_logger().info(f"Overhead Depth Generator started. Subscribed to {self.depth_topic}")

    def depth_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            # Gazebo depth images are usually 32FC1 (meters)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            if depth_image is None:
                return

            # Convert to numpy array
            depth_array = np.array(depth_image, dtype=np.float32)
            
            # Filter out NaN and Inf values
            depth_array = np.nan_to_num(depth_array, nan=0.0, posinf=10.0, neginf=0.0)
            
            # Constants for Table-Relative Height
            # Camera Z: 1.6m, Table Top Z: 0.4m -> Distance: 1.2m
            TABLE_DISTANCE = 1.20 
            
            # Calculate statistics for the HUD
            min_depth = np.min(depth_array[depth_array > 0]) if np.any(depth_array > 0) else 0.0
            max_depth = np.max(depth_array)
            mean_depth = np.mean(depth_array[depth_array > 0]) if np.any(depth_array > 0) else 0.0
            
            # Normalize for visualization (0 to 255)
            # Focus the dynamic range on the area between camera and table for better contrast
            norm_depth = cv2.normalize(np.clip(depth_array, 0.5, 1.3), None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # Apply Color Map for Rich Aesthetics
            depth_colormap = cv2.applyColorMap(norm_depth, cv2.COLORMAP_JET)
            
            # Object Detection based on Depth (Height from Table)
            # Anything closer than 1.15m is considered an object on the 1.2m table
            object_mask = (depth_array < (TABLE_DISTANCE - 0.02)) & (depth_array > 0.1)
            object_mask = object_mask.astype(np.uint8) * 255
            
            # Clean up mask
            kernel = np.ones((5,5), np.uint8)
            object_mask = cv2.morphologyEx(object_mask, cv2.MORPH_OPEN, kernel)
            
            contours, _ = cv2.findContours(object_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            detected_objects = []
            for cnt in contours:
                if cv2.contourArea(cnt) < 100: continue
                
                # Get average depth in this contour
                mask = np.zeros(depth_array.shape, np.uint8)
                cv2.drawContours(mask, [cnt], -1, 255, -1)
                avg_depth = cv2.mean(depth_array, mask=mask)[0]
                
                height_from_table = TABLE_DISTANCE - avg_depth
                
                # Centroid for label
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    detected_objects.append((cx, cy, height_from_table))
                    
                    # Draw contour and label on the colormap
                    cv2.drawContours(depth_colormap, [cnt], -1, (255, 255, 255), 1)
                    label = f"H: {height_from_table*100:.1f}cm"
                    cv2.putText(depth_colormap, label, (cx - 20, cy), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

            # Add Premium HUD Elements
            self.draw_hud(depth_colormap, min_depth, max_depth, mean_depth, len(detected_objects))
            
            # Show in a separate window
            cv2.imshow("Overhead Camera - Live Depth Map", depth_colormap)
            cv2.waitKey(1)
            
            self.frame_count += 1
            if self.frame_count % 10 == 0:
                curr_time = time.time()
                self.fps = 10 / (curr_time - self.last_time)
                self.last_time = curr_time

        except Exception as e:
            self.get_logger().error(f"Error in depth_callback: {e}")

    def draw_hud(self, img, min_d, max_d, mean_d, num_objs):
        h, w, _ = img.shape
        
        # Semi-transparent overlay for header
        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (w, 60), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, img, 0.4, 0, img)
        
        # Title
        cv2.putText(img, "OVERHEAD DEPTH ANALYZER", (20, 35), 
                    cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255, 255), 2)
        
        # Dynamic Stats
        stats_color = (0, 255, 255)
        cv2.putText(img, f"FPS: {self.fps:.1f} | Objects: {num_objs}", (w - 220, 35), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, stats_color, 1)
        
        # Bottom HUD
        cv2.rectangle(img, (0, h - 40), (w, h), (20, 20, 20), -1)
        cv2.putText(img, f"Min: {min_d:.2f}m | Max: {max_d:.2f}m | Mean: {mean_d:.2f}m", (20, h - 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Depth Scale Legend (Vertical)
        legend_w = 20
        legend_h = 150
        lx, ly = w - 40, h // 2 - legend_h // 2
        for i in range(legend_h):
            val = int(255 * (1 - i / legend_h))
            color = cv2.applyColorMap(np.array([[val]], dtype=np.uint8), cv2.COLORMAP_JET)[0, 0]
            cv2.line(img, (lx, ly + i), (lx + legend_w, ly + i), tuple(int(c) for c in color), 1)
        
        cv2.putText(img, "CLOSE", (lx - 45, ly + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
        cv2.putText(img, "FAR", (lx - 35, ly + legend_h - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)

def main(args=None):
    rclpy.init(args=args)
    node = OverheadDepthGenerator()
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

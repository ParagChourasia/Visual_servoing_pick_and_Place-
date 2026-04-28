#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from franka_sim_interfaces.srv import DetectObject

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        # Parameters
        self.declare_parameter('image_topic', '/wrist_camera/image')
        self.declare_parameter('depth_topic', '/wrist_camera/depth_image')
        
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        
        # ROS 2 Sub/Pub
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        
        # Service Server
        self.srv = self.create_service(DetectObject, 'detect_object', self.detect_object_callback)
        
        self.overlay_pub = self.create_publisher(Image, '/wrist_detector/overlay_image', 10)
        
        self.bridge = CvBridge()
        self.latest_bgr_img = None
        self.latest_depth_img = None
        
        # HSV Color Ranges
        self.colors = {
            'red':    {'lower': np.array([0, 100, 100]),   'upper': np.array([10, 255, 255]),  'id': 1.0, 'bgr': (0, 0, 255)},
            'green':  {'lower': np.array([35, 100, 100]),  'upper': np.array([85, 255, 255]),  'id': 2.0, 'bgr': (0, 255, 0)},
            'blue':   {'lower': np.array([100, 150, 0]),   'upper': np.array([140, 255, 255]), 'id': 3.0, 'bgr': (255, 0, 0)}
        }
        
        self.get_logger().info('Object Detector Service Node Initialized.')

    def depth_callback(self, msg):
        try:
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().error(f'Depth Bridge Error: {e}')

    def image_callback(self, msg):
        try:
            self.latest_bgr_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.publish_overlay(msg.header)
        except Exception as e:
            pass

    def detect_object_callback(self, request, response):
        color_name = request.color.lower()
        self.get_logger().info(f"Incoming request to detect: {color_name}")
        
        if self.latest_bgr_img is None or self.latest_depth_img is None:
            response.found = False
            return response

        if color_name not in self.colors:
            self.get_logger().warn(f"Unknown color: {color_name}")
            response.found = False
            return response

        data = self.colors[color_name]
        hsv = cv2.cvtColor(self.latest_bgr_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, data['lower'], data['upper'])
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 400:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX, cY = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                    
                    # Get Depth val (clamped)
                    h, w = self.latest_depth_img.shape
                    cX_cl, cY_cl = max(0, min(cX, w-1)), max(0, min(cY, h-1))
                    depth_val = self.latest_depth_img[cY_cl, cX_cl]
                    
                    if not np.isnan(depth_val) and depth_val > 0:
                        response.found = True
                        # We return pixel-based Z in the Pose message for high-level projection
                        # or we can do projection here if we had CameraInfo. 
                        # For now, following the existing 'pick_and_place.py' logic, 
                        # we return the pixel coordinates in the Pose object for simplicity 
                        # of the migration, but technically we should return world/camera coords.
                        # Wait, the action server will need world coords. 
                        # The existing pick_and_place.py uses point_to_world. 
                        # Let's keep the Service returning Point (u,v,depth) for projection in the Action Server.
                        
                        response.pose.header.stamp = self.get_clock().now().to_msg()
                        response.pose.header.frame_id = 'camera_link_optical'
                        response.pose.pose.position.x = float(cX)
                        response.pose.pose.position.y = float(cY)
                        response.pose.pose.position.z = float(depth_val)
                        return response

        response.found = False
        return response

    def publish_overlay(self, header):
        if self.latest_bgr_img is None: return
        
        cv_image = self.latest_bgr_img.copy()
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        any_detected = False
        for color_name, data in self.colors.items():
            mask = cv2.inRange(hsv, data['lower'], data['upper'])
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 400:
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), data['bgr'], 2)
                    cv2.putText(cv_image, color_name.upper(), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, data['bgr'], 2)
                    any_detected = True

        if not any_detected:
            cv2.putText(cv_image, "SCANNING...", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
        try:
            overlay_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            overlay_msg.header = header
            self.overlay_pub.publish(overlay_msg)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    detector = ObjectDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

def main(args=None):
    rclpy.init(args=args)
    detector = ObjectDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

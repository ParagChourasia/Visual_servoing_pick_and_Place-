#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        # Parameters
        self.declare_parameter('image_topic', '/wrist_camera/image')
        self.declare_parameter('depth_topic', '/wrist_camera/depth_image')
        self.declare_parameter('output_topic', '/detected_object_pixel')
        
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        
        # ROS 2 Sub/Pub
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        
        self.pixel_pub = self.create_publisher(Point, output_topic, 10)
        self.overlay_pub = self.create_publisher(Image, '/wrist_detector/overlay_image', 10)
        
        self.bridge = CvBridge()
        self.latest_depth_img = None
        
        # HSV Color Ranges (Tuned for Gazebo Fortress)
        self.colors = {
            'RED':    {'lower': np.array([0, 100, 100]),   'upper': np.array([10, 255, 255]),  'id': 1.0, 'bgr': (0, 0, 255)},
            'GREEN':  {'lower': np.array([35, 100, 100]),  'upper': np.array([85, 255, 255]),  'id': 2.0, 'bgr': (0, 255, 0)},
            'BLUE':   {'lower': np.array([100, 150, 0]),   'upper': np.array([140, 255, 255]), 'id': 3.0, 'bgr': (255, 0, 0)}
        }
        
        self.get_logger().info('Object Detector with RViz Overlay initialized.')

    def depth_callback(self, msg):
        try:
            # Depth in Gazebo is usually 32FC1 (meters)
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().error(f'Depth Bridge Error: {e}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        detected = False
        
        for color_name, data in self.colors.items():
            mask = cv2.inRange(hsv, data['lower'], data['upper'])
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 400:
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cX, cY = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                        
                        # Get Depth
                        depth_val = 0.0
                        if self.latest_depth_img is not None:
                            # Clamp cX, cY to image boundaries
                            h, w = self.latest_depth_img.shape
                            cX_cl = max(0, min(cX, w-1))
                            cY_cl = max(0, min(cY, h-1))
                            depth_val = self.latest_depth_img[cY_cl, cX_cl]
                            if np.isnan(depth_val): depth_val = 0.0

                        # Publish Result
                        point_msg = Point(x=float(cX), y=float(cY), z=data['id'])
                        self.pixel_pub.publish(point_msg)
                        
                        # RViz Overlay Dashboard
                        # 1. Box and circle
                        x, y, w, h = cv2.boundingRect(largest_contour)
                        cv2.rectangle(cv_image, (x, y), (x + w, y + h), data['bgr'], 2)
                        cv2.circle(cv_image, (cX, cY), 5, (255, 255, 255), -1)
                        
                        # 2. Text Box (Semi-transparent background for readability)
                        overlay_txt = f"{color_name} CUBE"
                        depth_txt = f"Depth: {depth_val:.3f}m"
                        
                        cv2.putText(cv_image, overlay_txt, (x, y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, data['bgr'], 2)
                        cv2.putText(cv_image, depth_txt, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                        
                        detected = True
                        break # Detect one primary object

        # Always publish the overlay image to RViz
        if not detected:
            cv2.putText(cv_image, "SEARCHING...", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
        try:
            overlay_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            overlay_msg.header = msg.header
            self.overlay_pub.publish(overlay_msg)
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    detector = ObjectDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

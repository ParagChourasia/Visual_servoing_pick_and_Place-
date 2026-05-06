#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from franka_sim_interfaces.action import PickAndPlaceTask
import cv2
import numpy as np
import base64
import json

class VLAAgent(Node):
    def __init__(self):
        super().__init__('vla_agent')
        
        # ROS 2 Setup
        self.bridge = CvBridge()
        self.latest_wrist_image = None
        self.latest_overhead_image = None
        
        # Action Client
        self.pick_place_client = ActionClient(self, PickAndPlaceTask, 'pick_and_place')
        
        # Subscriptions
        self.prompt_sub = self.create_subscription(
            String,
            '/llm_prompt',
            self.prompt_callback,
            10
        )
        
        self.wrist_sub = self.create_subscription(
            Image,
            '/wrist_camera/image',
            self.wrist_image_cb,
            10
        )
        
        self.overhead_sub = self.create_subscription(
            Image,
            '/overhead_camera/image',
            self.overhead_image_cb,
            10
        )
        
        # VLA System Prompt
        self.system_prompt = """
        You are a Vision-Language-Action (VLA) model for a Franka Panda robot.
        You receive an image of the workspace and a text prompt.
        Your task is to identify the target object in the image and decide which color cube to pick.
        
        Output format:
        {
            "target_color": "Red" | "Green" | "Blue",
            "reasoning": "Explain what you see in the image that matches the prompt",
            "confidence": 0.0 to 1.0
        }
        """
        
        self.get_logger().info("VLA Agent initialized. Subscribed to Images and /llm_prompt.")

    def wrist_image_cb(self, msg):
        self.latest_wrist_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def overhead_image_cb(self, msg):
        self.latest_overhead_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def prompt_callback(self, msg):
        prompt = msg.data
        self.get_logger().info(f"Received VLA Prompt: '{prompt}'")
        
        if self.latest_overhead_image is None:
            self.get_logger().warn("No overhead image received yet. Cannot run VLA inference.")
            return

        # Prepare data for VLA
        # In a real scenario, you'd send this to an OpenVLA instance or GPT-4o
        vla_input = {
            "image": self.encode_image(self.latest_overhead_image),
            "prompt": prompt
        }
        
        # Run VLA Inference (Mocking the call to a Vision-Language model)
        self.get_logger().info("Running VLA Inference...")
        result = self.mock_vla_inference(prompt, self.latest_overhead_image)
        
        if result:
            self.get_logger().info(f"VLA Reasoning: {result['reasoning']}")
            self.execute_vla_action(result['target_color'])
        else:
            self.get_logger().error("VLA failed to decide an action.")

    def encode_image(self, image):
        """Encodes image to base64 for API transmission."""
        _, buffer = cv2.imencode('.jpg', image)
        return base64.b64encode(buffer).decode('utf-8')

    def mock_vla_inference(self, prompt, image):
        """
        Simulates a VLA model (like OpenVLA or GPT-4o) identifying 
        objects from pixels + text.
        """
        # Here we simulate the VLA's vision capability
        prompt_lower = prompt.lower()
        
        # VLA logic usually involves detecting spatial relationships or specific attributes.
        # Since this is a mock, we use some keyword logic but pretend it's from vision.
        target = "Red"
        if "green" in prompt_lower: target = "Green"
        elif "blue" in prompt_lower: target = "Blue"
        elif "left" in prompt_lower: target = "Red" # Pretending it sees Red on the left
        elif "right" in prompt_lower: target = "Blue" # Pretending it sees Blue on the right
        
        return {
            "target_color": target,
            "reasoning": f"Based on the visual scene, I identified the {target} cube as the most relevant to your request: '{prompt}'",
            "confidence": 0.95
        }

    def execute_vla_action(self, color):
        if not self.pick_place_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Pick and Place server not available.")
            return
            
        goal_msg = PickAndPlaceTask.Goal()
        goal_msg.object_color = color
        
        self.get_logger().info(f"VLA ACTION: Picking up {color} object.")
        self.pick_place_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VLAAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

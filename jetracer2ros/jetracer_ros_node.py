import rclpy
from rclpy.node import Node

import torch
import torchvision
import numpy as np
import cv2
from cv_bridge import CvBridge
from PIL import Image

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

# Import the updated preprocess function
from jetracer2ros.utils import preprocess

import threading

class JetracerROSNode(Node):

    def __init__(self):
        super().__init__('jetracer_ros_node')
        self.subscriber = self.create_subscription(Image, '/camera/color/image_raw', self.subscriber_callback_camera, qos_profile_sensor_data)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.new_image = False
        self.cv_image = np.zeros((720, 1280, 3), np.uint8)  # Update shape to match camera resolution
        self.temp_angle = 0.0
        self.first_image = False

        # Load the model
        device = torch.device('cuda')

        # resnet
        self.model = torchvision.models.resnet18(pretrained=False)
        self.model.fc = torch.nn.Linear(512, 3)  # Adjust output layer for 3 classes

        # densenet
        # self.model = torchvision.models.densenet121(pretrained=False)
        # self.model.classifier = torch.nn.Linear(1024, 3)  # Adjust output layer for 3 classes


        self.model = self.model.to(device).eval().half()
        self.model.load_state_dict(torch.load('/home/af/jetracer_ws/src/jetracer2ros/road_following_model_resnet18_100.pth'))
        # self.model.load_state_dict(torch.load('/home/af/jetracer_ws/src/jetracer2ros/road_following_model_densenet121_100_2.pth'))

        # Parameters
        self.throttle = 0.5
        self.get_logger().info("jetracer_ros_node started.")

        # Create a thread for model inference
        self.model_thread = threading.Thread(target=self.model_inference_thread)
        self.model_thread.daemon = True
        self.model_thread.start()

    def model_inference_thread(self):
        while True:
            if self.new_image:
                image = preprocess(self.cv_image).half()
                output = self.model(image).detach().cpu().numpy().flatten()

                # Map the output to angles
                angle_mapping = {0: -30, 1: 0, 2: 30}
                class_idx = np.argmax(output)
                angle = angle_mapping[class_idx]

                # Store the angle
                self.temp_angle = float(angle)
                self.first_image = True

                # Reset new_image flag
                self.new_image = False

    def timer_callback(self):
        if self.first_image == True:
            msg = Twist()
            msg.linear.x = self.throttle
            msg.angular.z = self.temp_angle
            self.publisher_.publish(msg)

        self.get_logger().info(f"Steering Angle: {self.temp_angle} published")

    def subscriber_callback_camera(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.new_image = True
        self.get_logger().info(f"Camera Image Received")

def main(args=None):
    rclpy.init(args=args)

    jr_ros_node = JetracerROSNode()

    rclpy.spin(jr_ros_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    jr_ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

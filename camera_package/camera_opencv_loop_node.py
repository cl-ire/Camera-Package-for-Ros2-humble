import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
import time
import movement_control_util

from opencv import human_detector


class CameraOpencv(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.publisher_ = self.create_publisher(Int32MultiArray, '/position_data', 1)
        self.image_publisher = self.create_publisher(Image, '/opencv_image', 1)
        self.bridge = CvBridge()

        self.detector = human_detector.HumanDetector()

        self.time1
        self.loop()


    def loop(self):
        while True:
            try:
                self.time1 = movement_control_util.get_current_time()
                data = self.detector.locate_person()       # run opencv skript

                self.publish_data(data)
            except:
                self.get_logger().info('erro')

    def publish_data(self, data):
                
        try:            
            Position, return_image = data

            return_image_msg = self.bridge.cv2_to_imgmsg(return_image, "bgr8")    # Image Streamer
            self.image_publisher.publish(return_image_msg)
        except:
            self.get_logger().info('no Position data recived')

        if Position != []:
            Position[7] = self.time1
            Position[8] = movement_control_util.get_current_time()
            self.get_logger().info('Position data recived {}'.format(Position))
            position_data = Int32MultiArray()
            position_data.data = Position
            self.publisher_.publish(position_data)


def main(args=None):
    
    rclpy.init(args=args)
    camera_subscriber = CameraOpencv()  # create node object
    rclpy.spin(camera_subscriber)       # start node 

    camera_subscriber.destroy_node()    # destroy node
    rclpy.shutdown()


if __name__ == '__main__':
    main()

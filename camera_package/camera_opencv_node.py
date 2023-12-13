import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32MultiArray
from cv_bridge import CvBridge
import cv2
import time
from human_detector import human_detector

import os


class CameraOpencv(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        #create the subscriber 
        self.subscription = self.create_subscription(
            Image,                      #data type
            '/image_raw',               #topic published by v4l2_camera_node
            self.listener_callback,     #function to notify that a mesage was recived
            5)                          #queue size amount of the stored mesages  
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Int32MultiArray, '/position_data', 1)
        self.bridge = CvBridge()

        self.x = 1

        # self.get_logger().info(f"OpenCV Version: {cv2.__version__}")
        # self.get_logger().info(f"Available Attributes: {dir(cv2)}")

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test.jpg')
        self.frame = cv2.imread(path)
        # self.get_logger().info('read image {}'.format(self.frame.shape))
        self.detector = human_detector.HumanDetector()


    def listener_callback(self, Image):
        position_data = Int32MultiArray()
        
        self.get_logger().info('Image recived')      #consoll output to confirm that a mesage was recived 
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(Image, "bgr8")      #converts the ros image topic into the opencv image format
        except CvBridgeError as e:
            print(e)
        
        

        try:
            Position = self.detector.locate_person(self.frame)
            self.get_logger().info('Position data recived {}'.format(Position))

            
            # Position = [coordinate_x, coordinate_y, lenght_x, lenght_y, max_x, max_y]
            
            # test code
            # if self.x == 0:
            #     Position = [300,0,100,800,1280,960] #output variable
            #     self.x = 1
            # elif self.x == 1:
            #     Position = [0,150,100,800,1280,960] #output variable
            #     self.x = 2
            # elif self.x == 2:
            #     Position = [-300,0,100,800,1280,960] #output variable
            #     self.x = 3
            # elif self.x == 3:
            #     Position = [0,-150,100,800,1280,960] #output variable
            #     self.x = 4
            # elif self.x == 4:
            #     Position = [0,0,100,800,1280,960] #output variable
            #     self.x = 0
            
            position_data.data = Position

            self.publisher_.publish(position_data)
        except:
            self.get_logger().info('no Position data recived')


        time.sleep(1)



def main(args=None):
    

    rclpy.init(args=args)
    camera_subscriber = CameraOpencv()
    rclpy.spin(camera_subscriber)    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

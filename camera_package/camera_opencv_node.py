import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

from detect_person import DetectPerson      #imports our detect person function

class CameraOpencv(Node):
    def __init__(self, opencv_function):
        super().__init__('camera_subscriber')
        #create the subscriber 
        self.subscription = self.create_subscription(
            Image,                      #data type
            '/image_raw',               #topic published by v4l2_camera_node
            self.listener_callback,     #function to notify that a mesage was recived
            5)                          #queue size amount of the stored mesages  
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(String, 'topic', 1)
        self.bridge = CvBridge()

        self.opencv_function = opencv_function

    def listener_callback(self, Image):
        position_data = String()
        
        self.get_logger().info('Image recived')      #consoll output to confirm that a mesage was recived 
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(Image, "bgr8")      #converts the ros image topic into the opencv image format
        except CvBridgeError as e:
            print(e)
        
        Position = self.opencv_function.getPosition()

        position_data.data = str(Position[1]) + ''



        
    
   


def main(args=None):
    
    temp = 0,7      #später funktion die den faktor berechnet 
    opencv_function = DetectPerson(temp)

    rclpy.init(args=args)
    camera_subscriber = CameraOpencv(opencv_function)
    rclpy.spin(camera_subscriber)    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

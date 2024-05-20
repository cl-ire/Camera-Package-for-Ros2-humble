import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
import threading
import traceback
from opencv import movement_control_util

from opencv import human_detector_yolo, human_detector


# Define the thread that will continuously pull frames from the camera
class CameraBufferCleanerThread(threading.Thread):
    def __init__(self, camera, name='camera-buffer-cleaner-thread'):
        self.camera = camera
        self.last_frame = None
        super(CameraBufferCleanerThread, self).__init__(name=name)
        self.start()

    def run(self):
        while True:
            ret, self.last_frame = self.camera.read()


class CameraOpencv(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        self.declare_parameter('detector_type', "haarcascade")
        self.declare_parameter('timer_period', 0.5) # seconds
        self.declare_parameter('detector_path', "/home/ubuntu/ros2_ws/src/yolo_config/")

        self.declare_parameter('optimal_hight_percentage', 75)
    


        self.publisher_ = self.create_publisher(
            Int32MultiArray, '/position_data', 1)
        self.image_publisher = self.create_publisher(Image, '/opencv_image', 1)
        self.bridge = CvBridge()

        try:
            camera = cv2.VideoCapture(0)                    # Start the camera            
            self.vid0 = CameraBufferCleanerThread(camera)   # Start the cleaning thread
        except:
            pass

        if self.get_parameter('detector_type').value == "yolo":
            detector_path = self.get_parameter('detector_path').value
            optimal_hight_percentage = self.get_parameter('optimal_hight_percentage').value
            self.detector = human_detector_yolo.HumanDetector(path=detector_path, optimal_hight_percentage=optimal_hight_percentage)
            self.detector_active = True
        elif self.get_parameter('detector_type').value == "haarcascade":
            self.detector = human_detector.HumanDetector()
            self.detector_active = True
        elif self.get_parameter('detector_type').value == "none":
            self.detector_active = False


        timer_period = self.get_parameter('timer_period').value
        self.timer = self.create_timer(timer_period, self.loop)

    def loop(self):
        try:
            time1 = movement_control_util.get_current_time()
            Position = []

            # self.get_logger().info('Image shot at {}'.format(time1))
                        
            if self.vid0.last_frame is not None:
                image = self.vid0.last_frame

            if self.detector_active:
                value = self.detector.locate_person(image)       # run opencv skript
                Position, return_image = value
            else:
                return_image = image
                Position = []

            # self.get_logger().info('Image processed at {}'.format(movement_control_util.get_current_time()))
            # self.get_logger().info('Image published with data {}'.format(Position))

            self.get_logger().info('Image shot at: {} Image processed at: {}'.format(time1, movement_control_util.get_current_time()))
            self.publish_data(Position, time1, return_image)

        except Exception as e:
            self.get_logger().info('Error: %s' % str(e))            

    def publish_data(self, Position, time1, return_image):

        try:
            return_image_msg = self.bridge.cv2_to_imgmsg(return_image, "bgr8")    # Image Streamer
            self.image_publisher.publish(return_image_msg)

            if Position != []:
                while len(Position) < 12:
                    Position.append(0)
                               

                Position[8], Position[9]= movement_control_util.datetime_to_combined_int(time1)
                Position[10], Position[11] = movement_control_util.datetime_to_combined_int(movement_control_util.get_current_time())
                # self.get_logger().info('Position data recived {}'.format(Position))
                position_data = Int32MultiArray()
                position_data.data = Position
                self.publisher_.publish(position_data)

        except Exception as e:
            self.get_logger().info('Error in publish_data: %s' % str(e))
            traceback.print_exc()


def main(args=None):

    rclpy.init(args=args)
    camera_subscriber = CameraOpencv()  # create node object
    rclpy.spin(camera_subscriber)       # start node

    camera_subscriber.destroy_node()    # destroy node
    rclpy.shutdown()


if __name__ == '__main__':
    main()

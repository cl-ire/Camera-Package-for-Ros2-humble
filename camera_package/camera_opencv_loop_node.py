import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
import traceback
from opencv import movement_control_util

from opencv import human_detector_yolo, human_detector


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
            self.vid0 = cv2.VideoCapture(0)
            # self.vid0 = cv2.VideoCapture(0, cv2.CAP_V4L2)
            if self.vid0.set(cv2.CAP_PROP_BUFFERSIZE, 0):
                self.get_logger().info('Buffer size set to 0.')
            else:
                self.get_logger().info('Buffer size not set.')

            # self.vid0.set(cv2.CAP_PROP_FPS, 10)

            # actual_frame_rate = self.vid0.get(cv2.CAP_PROP_FPS)
            # actual_width = self.vid0.get(cv2.CAP_PROP_FRAME_WIDTH)
            # actual_height = self.vid0.get(cv2.CAP_PROP_FRAME_HEIGHT)
            # self.get_logger().info('frame rate: {} resolution: {} x {}'.format(actual_frame_rate, actual_width, actual_height))

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
            ret, image = self.vid0.read()
            ret, image = self.vid0.read()
            if self.detector_active:
                value = self.detector.locate_person(image)       # run opencv skript
                Position, return_image = value
            else:
                return_image = image
                Position = []

            # self.get_logger().info('Image processed at {}'.format(movement_control_util.get_current_time()))
            # self.get_logger().info('Image published with data {}'.format(Position))
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

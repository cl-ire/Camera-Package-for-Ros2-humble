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
        self.publisher_ = self.create_publisher(
            Int32MultiArray, '/position_data', 1)
        self.image_publisher = self.create_publisher(Image, '/opencv_image', 1)
        self.bridge = CvBridge()

        try:
            self.vid0 = cv2.VideoCapture(0)
            self.vid0.set(cv2.CAP_PROP_BUFFERSIZE, 0)
        except:
            pass

        # self.detector = human_detector_yolo.HumanDetector()
        self.detector = human_detector.HumanDetector()

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.loop)

    def loop(self):
        try:
            time1 = movement_control_util.get_current_time()
            Position = []

            # self.get_logger().info('Image shot at {}'.format(time1))
            ret, image = self.vid0.read()
            value = self.detector.locate_person(image)       # run opencv skript
            Position, return_image = value

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
                while len(Position) < 11:
                    Position.append(0)
                               

                Position[7], Position[8]= movement_control_util.datetime_to_combined_int(time1)
                Position[9], Position[10] = movement_control_util.datetime_to_combined_int(movement_control_util.get_current_time())
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

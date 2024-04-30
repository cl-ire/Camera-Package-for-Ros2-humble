import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
import time
import math
import datetime
from opencv.movement_control_util import calculate_movement_variable_time, set_timestamp, compare_times, add_seconds_to_time, get_current_time, calculate_angle, determine_percentage_of_height, aproximate_distance, combined_int_to_datetime

class MovementControl(Node):
    def __init__(self):
        super().__init__('movement_control')
        
        # parameters
        # camera angle
        self.max_winkel_x = 90
        self.max_winkel_y = 50
        # distance aproximation setings
        self.distance_to_person = 200
        self.hight_of_person = 170
        self.optimal_hight_percentage = determine_percentage_of_height(self)
        # movement settings Alphabot2
        # self.radius = 25
        # self.wheel_distance = 10
        # self.wheel_radius = 1.5
        # self.correction_factor = 1

        # movement settings Arduino
        self.radius = 25
        self.wheel_distance = 11
        self.wheel_radius = 3.5
        self.correction_factor = 1

        self.enable_movement = False

        # default time the vihical is moving       
        self.old_time = datetime.datetime.strptime("00:00:00.000000", "%H:%M:%S.%f").time()

        # Servo
        self.servo_msg_hold = [0, 0]
        self.servo_pub = self.create_publisher(Int32MultiArray, '/servo', 1)
        self.motor_msg = [0, 0, 0, 0, 0]
        self.motor_pub = self.create_publisher(Int32MultiArray, '/motor', 1)


    def loop(self):
        

        while True:
            time.sleep(1)
            self.position_callback(20, 50 , False)
            time.sleep(4)
            self.position_callback(-20, 50 , False)
            time.sleep(4)
            self.position_callback(0, 50 , True)
                

    def position_callback(self, angle, base_rpm, move):
        
        base_rpm = 50
        move = False
        speed_right, speed_left, time_out = calculate_movement_variable_time(self, base_rpm,angle, move)

        self.motor_msg[0] = speed_right    #rpm right motor
        self.motor_msg[1] = speed_left     #rpm left motor
        self.motor_msg[2] = int(time_out*1000)       #time the motors will run
        self.motor_msg[3] = 0              #rpm right motor after time_out
        self.motor_msg[4] = 0              #rpm left motor after time_out

        self.get_logger().info("Motor data: {}".format(self.motor_msg))


        motor_msg_sent = Int32MultiArray()
        motor_msg_sent.data = self.motor_msg
        self.motor_pub.publish(motor_msg_sent)
        # self.get_logger().info("Data sent to Motor: {}".format(self.motor_msg))

    
    
def main(args=None):
    
    rclpy.init(args=args)
    movement_control = MovementControl()  # create node object
    rclpy.spin(movement_control)       # start node
    
    movement_control.destroy_node()    # destroy node
    rclpy.shutdown()


if __name__ == '__main__':
    main()

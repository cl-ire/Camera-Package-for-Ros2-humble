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

        self.declare_parameter('camera_max_winkel_x', 90)
        self.declare_parameter('camera_max_winkel_y', 50)
        self.declare_parameter('distance_to_person', 200)
        self.declare_parameter('hight_of_person', 170)
        self.declare_parameter('motor_settings_radius', 25)
        self.declare_parameter('motor_settings_wheel_distance', 11)
        self.declare_parameter('motor_settings_wheel_radius', 3.5)
        self.declare_parameter('motor_settings_correction_factor', 1)
        self.declare_parameter('motor_settings_base_rpm', 50)        
        self.declare_parameter('enable_movement', False)
        

        self.max_winkel_x = self.get_parameter('camera_max_winkel_x').value
        self.max_winkel_y = self.get_parameter('camera_max_winkel_y').value
        self.distance_to_person = self.get_parameter('distance_to_person').value
        self.hight_of_person = self.get_parameter('hight_of_person').value
        self.optimal_hight_percentage = determine_percentage_of_height(self)
        self.radius = self.get_parameter('motor_settings_radius').value
        self.wheel_distance = self.get_parameter('motor_settings_wheel_distance').value
        self.wheel_radius = self.get_parameter('motor_settings_wheel_radius').value
        self.correction_factor = self.get_parameter('motor_settings_correction_factor').value
        self.base_rpm = self.get_parameter('motor_settings_base_rpm').value

        self.enable_movement = self.get_parameter('enable_movement').value

        # default time the vihical is moving       
        self.old_time = datetime.datetime.strptime("00:00:00.000000", "%H:%M:%S.%f").time()

        #Folow me 
        self.subscription = self.create_subscription(
            Int32MultiArray,            #data type
            '/position_data',           #topic published by camera_opencv_node
            self.position_callback,        #function to notify that a mesage was recived
            1)                          #queue size amount of the stored mesages  
        self.subscription
        self.Position = []

        #Joistick
        self.subscription = self.create_subscription(
            String,                     #data type
            '/joystick',                 #topic published by camera_opencv_node
            self.control_joystick,               #function to notify that a mesage was recived
            5)                          #queue size amount of the stored mesages  
        self.subscription

        #Control
        self.subscription = self.create_subscription(
            String,                     #data type
            '/control',                 #topic published by web_control_center_node
            self.control,               #function to notify that a mesage was recived
            5)                          #queue size amount of the stored mesages  
        self.subscription

        # Servo
        self.servo_msg_hold = [0, 0]
        self.servo_pub = self.create_publisher(Int32MultiArray, '/servo', 1)
        self.motor_msg = [0, 0, 0, 0, 0]
        self.motor_pub = self.create_publisher(Int32MultiArray, '/motor', 1)
                

    def position_callback(self, Position):
        
        self.coordinate_x = Position.data[0]
        self.coordinate_y = Position.data[1]
        self.lenght_x     = Position.data[2]
        self.lenght_y     = Position.data[3]
        self.max_x        = Position.data[4]
        self.max_y        = Position.data[5]
        self.time1        = combined_int_to_datetime(Position.data[7], Position.data[8])
        self.time2        = combined_int_to_datetime(Position.data[9], Position.data[10])

        self.get_logger().info('Position data recived {}'.format(Position.data))
        
        
        use_data, old_time = compare_times(self.time1, self.time2, self.old_time)

        if use_data:           

            self.servo_msg_hold, self.winkel_x, self.winkel_y = calculate_angle(self)

            if self.enable_movement:
                servo_msg_sent = Int32MultiArray()
                servo_msg_sent.data = self.servo_msg_hold
                self.servo_pub.publish(servo_msg_sent)
                self.get_logger().info("Data sent to Servo: {}".format(self.servo_msg_hold))

            try:                
                distance = aproximate_distance(self, self.lenght_y)
                # self.get_logger().info("Distance to person : {}".format(distance))
            except:
                self.get_logger().info("unable to calculate Distance")

            if self.winkel_x > 0 and self.winkel_x > 5 or self.winkel_x < 0 and self.winkel_x < -5:
                move = False
                speed_right, speed_left, time_out = calculate_movement_variable_time(self, self.base_rpm,self.winkel_x, move)

                self.motor_msg[0] = speed_right    #rpm right motor
                self.motor_msg[1] = speed_left     #rpm left motor
                self.motor_msg[2] = int(time_out*1000)       #time the motors will run
                self.motor_msg[3] = 0              #rpm right motor after time_out
                self.motor_msg[4] = 0              #rpm left motor after time_out

                self.get_logger().info("Motor data: {}".format(self.motor_msg))

                if self.enable_movement:
                    motor_msg_sent = Int32MultiArray()
                    motor_msg_sent.data = self.motor_msg
                    self.motor_pub.publish(motor_msg_sent)
                    self.get_logger().info("Data sent to Motor: {}".format(self.motor_msg))
                self.old_time = add_seconds_to_time(old_time, time_out + 0.15)
            
        # else:
        #     self.get_logger().info("Data not used old time: {} - new time {}".format(self.old_time, self.time1))
    

    def control_joystick(self, msg):
        #Joistick
        imput = msg.data
        self.get_logger().info("Joistick Imput recived: {}".format(imput))        
                
        if imput == "Center":
            # togle movement
            self.enable_movement = not self.enable_movement
            if self.enable_movement:
                self.get_logger().info("Movement enabled")
            else:
                self.get_logger().info("Movement diabled")

        elif imput == "Up":
            #center servo
            servo_msg_sent = Int32MultiArray()
            self.get_logger().info("Data sent to Servo: {}".format(self.servo_msg_hold))
            servo_msg_sent.data = [0, 0]
            self.servo_pub.publish(servo_msg_sent)
            time.sleep(1)
        # elif imput == "Right":
        #     #
        # elif imput == "Left":
        #     #
        # elif imput == "Down":
        #     #
    
    def control(self, msg):
        
        imput = msg.data
        self.get_logger().info("Control Imput recived: {}".format(imput))        
                
        if imput == "toggle_movement":
            # togle movement
            self.enable_movement = not self.enable_movement
            if self.enable_movement:
                self.get_logger().info("Movement enabled")
            else:
                self.get_logger().info("Movement diabled")

        elif imput == "center_servo":
            #center servo
            servo_msg_sent = Int32MultiArray()
            self.get_logger().info("Data sent to Servo: {}".format(self.servo_msg_hold))
            servo_msg_sent.data = [0, 0]
            self.servo_pub.publish(servo_msg_sent)
            time.sleep(1)
        
    
    
def main(args=None):
    
    rclpy.init(args=args)
    movement_control = MovementControl()  # create node object
    rclpy.spin(movement_control)       # start node
    
    movement_control.destroy_node()    # destroy node
    rclpy.shutdown()


if __name__ == '__main__':
    main()

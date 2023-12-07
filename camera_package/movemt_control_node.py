import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import Twist

import math

class MovementControl(Node):
    def __init__(self):
        super().__init__('movement_control')
        
        #Folow me 
        self.subscription = self.create_subscription(
            Int32MultiArray,            #data type
            '/position_data',           #topic published by camera_opencv_node
            self.listener_callback,     #function to notify that a mesage was recived
            5)                          #queue size amount of the stored mesages  
        self.subscription
        self.Position = []

        #Joistick
        self.subscription = self.create_subscription(
            String,                     #data type
            'joystick',                 #topic published by camera_opencv_node
            self.control,               #function to notify that a mesage was recived
            5)                          #queue size amount of the stored mesages  
        self.subscription

        # Servo
        self.servo_msg_hold = [0, 0]
        self.servo_pub = self.create_publisher(Int32MultiArray, '/Servo', 4)
        

    def listener_callback(self, msg):
        
        # channel 0 = pan 
		# channel 1 = tilt

        self.get_logger().info('Position recived')      #consoll output to confirm that a mesage was recived 
        self.Position.append(msg.data)         #save recived msg in a array

        self.coordinate_x = self.Position[1]
        self.coordinate_y = self.Position[2]
        self.lenght_x     = self.Position[3]
        self.lenght_y     = self.Position[4]
        self.max_x        = self.Position[5]
        self.max_y        = self.Position[6]
        self.max_winkel_x = 66
        self.max_winkel_y = 48
        
        # Winkelberechnung
        self.winkel_x = int((self.center_x/self.max_x)*self.max_winkel_x)
        self.winkel_y = int((self.center_y/self.max_y)*self.max_winkel_y)
        
        self.servo_msg_hold[0] = self.servo_msg_hold[0] + self.winkel_x
        self.servo_msg_hold[1] = self.servo_msg_hold[1] + self.winkel_y

        servo_msg_sent = Int32MultiArray()
        
        self.get_logger().info("Data sent to Servo: {}".format(self.servo_msg_hold))
        servo_msg_sent.data = self.servo_msg_hold
        self.servo_pub.publish(servo_msg_sent)

        # self.determine_percentage_of_height(200, 180, 48)
        

        time.sleep(0.2)

    def control(self, msg):
        #Joistick
        imput = msg.data
        self.get_logger().info("Joistick Imput recived: {}".format(imput)) 
        
                
        # if imput == "Center":
        #     #
        # elif imput == "Up":
        #     #
        # elif imput == "Right":
        #     #
        # elif imput == "Left":
        #     #
        # elif imput == "Down":
        #     #

    

    # def determine_percentage_of_height(wanted_distance, height, angle):
    #     a = wanted_distance
    #     angle_rad = math.radians(angle)
    #     #print(wanted_distance, height, angle, angle_rad)

    #     b = round(a * math.tan(angle_rad))
    #     #print(b)


    #     required_percentage = round((height/b) * 100)

    #     return required_percentage



def main(args=None):
    

    rclpy.init(args=args)
    movement_control = MovementControl()
    rclpy.spin(movement_control)    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    movement_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

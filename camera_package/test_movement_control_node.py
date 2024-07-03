import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray

import time
from opencv.movement_control_util import datetime_to_combined_int, get_current_time, add_seconds_to_time
import json
import numpy

class TestData:
    def __init__(self, position_data, servo_expected_results, motor_expected_results, time0, number):
        self.position_data = position_data
        self.servo_expected_results = servo_expected_results
        self.motor_expected_results = motor_expected_results
        self.servo_received_results = []
        self.motor_received_results = []
        self.servo_test_success = False
        self.motor_test_success = False
        
        self.number = number
        
        time1, time2 = datetime_to_combined_int(time0)
        self.position_data.append(time1)
        self.position_data.append(time2)
        
        time0 = add_seconds_to_time(time0, 2)
        time1, time2 = datetime_to_combined_int(time0)
        self.position_data.append(time1)
        self.position_data.append(time2)
        
    def to_json(self):
        return {
            'position_data': list(self.position_data),
            'servo_expected_results': list(self.servo_expected_results),
            'motor_expected_results': list(self.motor_expected_results),
            'servo_received_results': list(self.servo_received_results),
            'motor_received_results': list(self.motor_received_results),
            'servo_test_success': self.servo_test_success,
            'motor_test_success': self.motor_test_success
        }
        
    def to_string(self):        
        string = '''
        \nnumber: {}
        position_data: {}
        servo_expected_results: {}
        motor_expected_results: {}
        servo_received_results: {}
        motor_received_results: {}
        servo_test_success: {}
        motor_test_success: {}
        '''.format(self.number,
                list(self.position_data), 
                list(self.servo_expected_results), 
                list(self.motor_expected_results), 
                list(self.servo_received_results), 
                list(self.motor_received_results), 
                self.servo_test_success, 
                self.motor_test_success)
        return string
    
    
class TestMovementControl(Node):
    def __init__(self):
        super().__init__('test_movement_control')

                
        self.subscription = self.create_subscription(
            Int32MultiArray,            #data type
            '/servo',           #topic published by camera_opencv_node
            self.servo_callback,        #function to notify that a mesage was recived
            1)                          #queue size amount of the stored mesages  
        self.subscription

        self.subscription = self.create_subscription(
            Int32MultiArray,                     #data type
            '/motor',                 #topic published by camera_opencv_node
            self.motor_callback,               #function to notify that a mesage was recived
            5)                          #queue size amount of the stored mesages  
        self.subscription
        
        self.position_pub = self.create_publisher(Int32MultiArray, '/position_data', 1)
        
        self.skip_servo_test = False
        self.counter = 0
        self.test_data_list = []
        self.setup()
        
        time.sleep(3)
        
        self.send_position()
        
    def setup(self):
        
        time0 = get_current_time()
        self.test_data_list.append(TestData(
            [-693, 158, 162, 626, 1920, 1080, 1, 0],
            [0, -30],
            [131, 100, 499, 0, 0],
            time0,
            0
            ))
        time0 = add_seconds_to_time(time0, 9)
        self.test_data_list.append(TestData(
            [200, 100, 100, 500, 1920, 1080, 1, 0],
            [0, -22],
            [100, 131, 133, 0, 0],
            time0,
            1
            ))
        time0 = add_seconds_to_time(time0, 9)
        self.test_data_list.append(TestData(
            [1000, 50, 170, 700, 1920, 1080, 1, 0],
            [0, 22],
            [100, 131, 733, 0, 0],
            time0,
            2
            ))
        time0 = add_seconds_to_time(time0, 9)
        self.test_data_list.append(TestData(
            [-300, -10, 100, 500, 1920, 1080, 1, 0],
            [0, 9],
            [131, 100, 216, 0, 0],
            time0,
            3
            ))
        time0 = add_seconds_to_time(time0, 9)
        self.test_data_list.append(TestData(
            [0, 0, 100, 500, 1920, 1080, 1, 0],
            [0, 9],
            [100, 100, 1000, 0, 0],
            time0,
            4
            ))
        
    
    def send_position(self):
        
        self.get_logger().info("Sending position: " + str(self.counter))
        
        position_msg = Int32MultiArray()
        position_msg.data = self.test_data_list[self.counter].position_data
        self.position_pub.publish(position_msg)
    
    
    def servo_callback(self, msg):
          
        self.test_data_list[self.counter].servo_received_results = msg.data
        
        if numpy.array_equal(self.test_data_list[self.counter].servo_expected_results,
                             self.test_data_list[self.counter].servo_received_results):
            self.test_data_list[self.counter].servo_test_success = True
        
        if self.counter +1 >= len(self.test_data_list):
            self.evaluate_test()
            return
        
        if len(self.test_data_list[self.counter].motor_received_results) > 0:
            self.counter += 1            
            self.send_position()
            
        
    def motor_callback(self, msg):
        
        self.get_logger().info("Motor callback: " + str(self.counter))
        self.test_data_list[self.counter].motor_received_results = msg.data
        
        if numpy.array_equal(self.test_data_list[self.counter].motor_expected_results,
                             self.test_data_list[self.counter].motor_received_results):
            self.test_data_list[self.counter].motor_test_success = True
        
        if self.counter +1 >= len(self.test_data_list):
            self.evaluate_test()
            return
        
        if len(self.test_data_list[self.counter].servo_received_results) > 0 or self.skip_servo_test:
            self.counter += 1            
            self.send_position()
            
    def evaluate_test(self):
        if self.skip_servo_test:
            self.get_logger().info("Servo test skipped")
        
        # test_data_str = json.dumps([obj.to_json() for obj in self.test_data_list])
        test_data_str = ""
        for test_data in self.test_data_list:
            test_data_str += test_data.to_string()
        
        string = ("\n" + test_data_str)
        self.get_logger().info(string)
        
        
    
def main(args=None):
    
    rclpy.init(args=args)
    test_movement_control = TestMovementControl()  # create node object
    rclpy.spin(test_movement_control)       # start node
    
    test_movement_control.destroy_node()    # destroy node
    rclpy.shutdown()


if __name__ == '__main__':
    main()
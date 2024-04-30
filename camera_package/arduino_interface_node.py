import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import serial.tools.list_ports
import glob
import time


class Serial_Arduino():
    def __init__(self):
        # Define the serial port and baud rate
        # self.serial_port = self.find_arduino_port()  # Change this to the appropriate port on your system
        self.find_arduino_port()
        self.serial_port = self.find_arduino_port()
        baud_rate = 9600
        self.ser = serial.Serial(self.serial_port, baud_rate, timeout=1)
        time.sleep(2)
        self.ser.flush()

    def find_arduino_port(self):
        arduino_ports = [
            p.device
            for p in serial.tools.list_ports.comports()
            if 'Arduino' in p.description or 'VID:PID' in p.hwid
        ]
        if arduino_ports:
            return arduino_ports[0]  # Assuming there's only one Arduino connected
        else:
            return None

    def write(self, data):
        self.ser.write(data.encode())
        self.ser.flush()
        # self.ser.write(str(data))

    def read(self):
        return self.ser.readline()



class ArduinoInterface(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        #create the subscriber 
        self.subscription = self.create_subscription(
            Int32MultiArray,                      #data type
            '/motor',               #topic published by v4l2_camera_node
            self.listener_callback,     #function to notify that a mesage was recived
            1)                          #queue size amount of the stored mesages  
        self.subscription               # prevent unused variable warning
        
        self.get_logger().info('Arduino interface started')
        self.ser = Serial_Arduino()
        self.get_logger().info('Arduino interface started on port {}'.format(self.ser.serial_port))


    def listener_callback(self, msg):
        self.get_logger().info('Motor data recived {}'.format(msg.data))

        self.speed_right = msg.data[0]
        self.speed_left = msg.data[1]
        self.speed_time = msg.data[2]
        self.speed_right_after = msg.data[3]
        self.speed_left_after = msg.data[4]

        serial_data = "{},{},{},{},{}\n".format(self.speed_right, self.speed_left, self.speed_time, self.speed_right_after, self.speed_left_after)
        self.get_logger().info('Serial data sent {}'.format(serial_data))

        self.ser.write(serial_data)
        time.sleep(0.2)
        response = self.ser.read()
        self.get_logger().info('Arduino response: {}'.format(response))



        

def main(args=None):

    rclpy.init(args=args)
    arduino_interface = ArduinoInterface()  # create node object
    rclpy.spin(arduino_interface)       # start node 

    arduino_interface.destroy_node()    # destroy node
    rclpy.shutdown()


if __name__ == '__main__':
    main()
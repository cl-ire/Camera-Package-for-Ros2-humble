import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import socket
import threading
import subprocess

current_directory = os.path.dirname(__file__)
share_directory = os.path.abspath(os.path.join(current_directory, '..', '..', '..', '..', 'share/camera_package/camera_package'))
templates_folder = 'views'
static_folder = 'static'
templates_path = os.path.join(share_directory, templates_folder)
static_path = os.path.join(share_directory, static_folder)
print("Templates path:", templates_path, "Static path:", static_path)

app = Flask(__name__, template_folder=templates_path, static_folder=static_path)
app.debug = True
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")
print("Template folder path:", app.template_folder)
print("Static folder path:", app.static_folder)

class WebControlCenter(Node):
    def __init__(self):
        super().__init__('image_streamer')
        self.subscription = self.create_subscription(
            Image,
            '/opencv_image',  # Replace with your image topic
            self.image_callback,
            10  # Adjust the queue size based on your needs
        )
        self.subscription  # Prevent unused variable warning
        self.bridge = CvBridge()

        self.control_pub = self.create_publisher(String, '/control', 3)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        _, buffer = cv2.imencode('.jpg', cv_image)
        image_data = buffer.tobytes()
        app.image_data = image_data

def generate():
    while True:
        if hasattr(app, 'image_data'):
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + app.image_data + b'\r\n')

@app.route('/')
def index():
    print("Index route hit")
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    print("Video feed route hit")
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/button_click/<button_name>', methods=['POST'])
def on_button_click(self, button_name):
    # Your Python code here
    print(f'Button {button_name} clicked')

    try:
        control_msg = String()
        control_msg.data = button_name
        self.control_pub.publish(control_msg)
    except:
        print('Error publishing control message')

    if button_name == 'launch_follow_me':
        print('launching follow me')
        print_message('launching follow me')
        

    return ('', 204)

@app.route('/print_message/<message>', methods=['POST'])
def print_message(message):
    socketio.emit('new_message', {'message': message})
    return ('', 204)


def launch_follow_me():
    launch_file = "follow_me_web_launch.py"    
    print('launching follow me')
    print_message('launching follow me')

    current_dir = os.path.dirname(os.path.realpath(__file__))
    relative_path_to_launch_file = os.path.join(current_dir, '..', 'launch', launch_file)

    subprocess.Popen(["ros2", "launch", relative_path_to_launch_file])



def main():
    rclpy.init()
    image_streamer = WebControlCenter()
    
    port = 5000
    local_ip = socket.gethostbyname(socket.gethostname())
    print('--------------------------------------------')
    print(f" access video feed on http://{local_ip}:{port}")
    print('--------------------------------------------')

    socketio.run(app, host=local_ip, port=port, allow_unsafe_werkzeug=True)

    try:
        rclpy.spin(image_streamer)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    

if __name__ == '__main__':
    main()

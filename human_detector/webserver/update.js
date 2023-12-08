// Connect to ROS
const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'  // Replace with your rosbridge_websocket server address
  });
  
  // Create an image listener
  const listener = new ROSLIB.Topic({
    ros: ros,
    name: '/image_raw',  // Replace with your image topic
    messageType: 'sensor_msgs/Image'
  });
  
  // Image element
  const image = document.getElementById('image-stream');
  
  // Subscribe to the image topic
  listener.subscribe(function (message) {
    // Update the image source with the received data
    image.src = 'data:image/jpeg;base64,' + message.data;
  });
  
  // Handle ROS connection events
  ros.on('connection', function () {
    console.log('Connected to ROS');
  });
  
  ros.on('error', function (error) {
    console.log('Error connecting to ROS:', error);
  });
  
  ros.on('close', function () {
    console.log('Connection to ROS closed');
  });
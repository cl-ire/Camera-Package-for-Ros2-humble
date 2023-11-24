#! /usr/bin/env python2

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import message_filters

def read_cameras():
    imageL = message_filters.Subscriber("/camera/left/image_raw", Image)
    imageR = message_filters.Subscriber("/camera/right/image_raw", Image)

    # Synchronize images
    ts = message_filters.ApproximateTimeSynchronizer([imageL, imageR], queue_size=10, slop=0.5)
    ts.registerCallback(image_callback)
    rospy.spin()

def image_callback(imageL, imageR):
    br = CvBridge()
    rospy.loginfo("receiving frame")
    imageLeft = br.imgmsg_to_cv2(imageL)
    imageRight = br.imgmsg_to_cv2(imageR)
    # Process images...

if __name__ == '__main__':
    rospy.init_node('my_node')
    try:
        read_cameras()
    except rospy.ROSInterruptException:
        pass
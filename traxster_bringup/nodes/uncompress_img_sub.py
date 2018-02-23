#!/usr/bin/env python
import sys, time
import numpy as np
#from scipy.ndimage import filters
import cv2
import roslib
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class image_feature:
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)

        self.subscriber = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.callback,  queue_size = 1)

        self.bridge = CvBridge()

    def callback(self, ros_data):
        '''Callback function of subscribed topic.  Here images get converted and features detected'''
        
        #np_arr = np.fromstring(ros_data.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        image_np = self.bridge.compressed_imgmsg_to_cv2(ros_data, desired_encoding='passthrough')

        # Publish new image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(image_np))

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('uncompress_image', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
#!/usr/bin/env python
import sys

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class camera_viewer:
    def __init__(self):
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)

def main(args):
    camera_viewer()
    rospy.init_node('camera_viewer')
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)
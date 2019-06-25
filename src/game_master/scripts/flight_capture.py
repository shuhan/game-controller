#!/usr/bin/env python
import sys
import os
import datetime

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class flight_capture:
    def __init__(self):
        
        self.data_path = os.path.join(os.path.expanduser('~'), 'flight_capture')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback)
        if not os.path.isdir(self.data_path):
            os.makedirs(self.data_path)


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            file_name = datetime.datetime.fromtimestamp(data.header.stamp.secs).strftime('%Y-%m-%d-%H:%M:%S.png')
            cv2.imwrite(os.path.join(self.data_path, file_name), cv_image)

            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)

def main(args):
    flight_capture()
    rospy.init_node('flight_capture')
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)
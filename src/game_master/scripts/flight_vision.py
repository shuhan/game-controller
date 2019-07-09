#!/usr/bin/env python
import sys
import os
import datetime

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class flight_capture:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            origImg =  self.bridge.imgmsg_to_cv2(data, "bgr8")

            im = np.float32(origImg) / 255.0

            # Calculate gradient 
            gx = cv2.Sobel(im, cv2.CV_32F, 1, 0, ksize=1)
            gy = cv2.Sobel(im, cv2.CV_32F, 0, 1, ksize=1)

            mag, angle = cv2.cartToPolar(gx, gy, angleInDegrees=True)

            ret,thresh = cv2.threshold(cv2.cvtColor(mag * 255, cv2.COLOR_BGR2GRAY), 10, 255, cv2.THRESH_BINARY)

            edges = cv2.Canny(np.uint8(thresh), 100, 200, apertureSize = 3)

            lines = cv2.HoughLines(edges, 1, np.pi/180, 200)

            height, width, channels = origImg.shape

            leftLine    = None
            rightLine   = None

            if lines is not None:
                print("Lines found: ", lines.shape)
                for line in lines:
                    for rho,theta in line:
                        a = np.cos(theta)
                        b = np.sin(theta)

                        if(a == 0 or b == 0):
                            continue

                        points = []

                        # Check x = 0 crossing
                        x1 = 0
                        y1 = int(rho/b)
                        if y1 >= 0 and y1 <= height:
                            points.append((x1, y1))

                        # Check y = 0 crossing
                        x1 = int(rho/a)
                        y1 = 0
                        if x1 >=0 and x1 <= width:
                            points.append((x1, y1))

                        # Check x = width
                        x1 = width
                        y1 = int((rho - a*width)/b)
                        if y1 >= 0 and y1 <= height:
                            points.append((x1, y1))

                        # Check y = height
                        x1 = int((rho - (b*height))/a)
                        y1 = height
                        if x1 >=0 and x1 <= width:
                            points.append((x1, y1))

                        if len(points) < 2:
                            continue

                        x1 = points[0][0]
                        y1 = points[0][1]
                        x2 = points[1][0]
                        y2 = points[1][1]

                        if leftLine is None:
                            if (x1 < width/2 or x2 < width/2):
                                leftLine = [(x1, y1), (x2, y2)]
                        else:
                            if (((y1 > leftLine[0][1] and y1 > leftLine[1][1] and x1 < width/2) or (y2 > leftLine[0][1] and y2 > leftLine[1][1] and x2 < width/2))):
                                leftLine = [(x1, y1), (x2, y2)]
                            elif (x1 > leftLine[0][0] and x1 > leftLine[1][0] and y1 == height and x1 < width/2) or (x2 > leftLine[0][0] and x2 > leftLine[1][0] and y2 == height and x2 < width/2):
                                leftLine = [(x1, y1), (x2, y2)]


                        if rightLine is None:
                            if (x1 >= width/2 or x2 <= width):
                                rightLine = [(x1, y1), (x2, y2)]
                        else:
                            if (((y1 > rightLine[0][1] and y1 > rightLine[1][1] and x1 >= width/2) or (y2 > rightLine[0][1] and y2 > rightLine[1][1] and x2 >= width/2))):
                                rightLine = [(x1, y1), (x2, y2)]
                            elif (x1 < rightLine[0][0] and x1 < rightLine[1][0] and y1 == height and x1 >= width/2) or (x2 < rightLine[0][0] and x2 < rightLine[1][0] and y2 == height and x2 >= width/2):
                                rightLine = [(x1, y1), (x2, y2)]

                        cv2.line(origImg, (x1,y1), (x2,y2), (0,0,255), 2)
            else:
                print("No line detected")

            if leftLine is not None:
                cv2.line(origImg, leftLine[0], leftLine[1], (255,0,0), 2)

            if rightLine is not None:
                cv2.line(origImg, rightLine[0], rightLine[1], (0,255,0), 2)

            cv2.imshow('Orig', origImg)
            cv2.imshow('Edges', edges)
            cv2.imshow('Magnitude', thresh)
            cv2.waitKey(1)


        except CvBridgeError as e:
            print(e)

def main(args):
    flight_capture()
    rospy.init_node('flight_vision')
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)
#!/usr/bin/env python
import sys
import os
import datetime
import time

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError
from object_detector import Detector

class DroneVision:

    def __init__(self, drone, vfov=45, hfov=80, expectedDistance = 6.50):
        self.drone              = drone
        self.vertical_fov       = vfov
        self.horizontal_fov     = hfov
        self.expectedDistance   = expectedDistance
        self.detector           = Detector(drone, vfov, hfov)
        self.frameTime          = 0

    def calculateFrontalDistance(self, origImg, frameTime, Display=True):

        # Navigate
        height, _, _                        = origImg.shape
        guideLine, guideTheta               = self.findFrontGuide(origImg, frameTime, True)
        
        # Find MR York
        self.detector.findMrYork(origImg, frameTime, Display)

        # Always update the guide line here
        self.drone.guideLine                = guideLine

        if guideLine is not None:
            # Calculate distance to front guide
            yvals                           = np.array([float(guideLine[0][1]), float(guideLine[1][1])])
            phi                             = (self.vertical_fov/2) - (((height - yvals) / height) * self.vertical_fov)
            theta                           = self.drone.camera_tilt
            zeta                            = np.degrees(self.drone.pitch)
            angle                           = np.radians(90 - (phi - theta - zeta))
            distance                        = self.drone.altitude * np.tan(angle)
            averageDistance                 = np.average(distance)
            if averageDistance < 8.0:       # Ignore if the distance is out of bound
                self.drone.guideLine        = guideLine
                self.drone.guideTheta       = guideTheta
                self.drone.guideAngularError= np.radians(90) - guideTheta
                self.drone.goodGuide        = averageDistance >= self.expectedDistance - 0.5 and averageDistance <= self.expectedDistance + 0.5
                self.drone.setDistance(averageDistance)

            # Evaluate if guide is good
            # A good guide gives rough estimation with 20 cm error
            # How do we determine if the guide is a good guide

            if self.frameTime > 0 and self.drone.state != self.drone.FLIGHT_STATE_NOT_FLYING:
                self.expectedDistance = self.expectedDistance - (self.drone.speedX * (frameTime - self.frameTime))
                self.frameTime = self.frameTime - frameTime

            if Display:
                font = cv2.FONT_HERSHEY_SIMPLEX
                line_color = (0, 255, 0)
                if not self.drone.goodGuide:
                    line_color = (0, 0, 255)
                else:
                    if averageDistance < self.expectedDistance:
                        self.expectedDistance = averageDistance
                cv2.line(origImg, guideLine[0], guideLine[1], line_color, 2)
                cv2.putText(origImg,'%.2f : %.2f - %.2f : %.2f - %.2f' % (distance[0], distance[1], averageDistance, self.expectedDistance, self.drone.speedX), (10,50), font, 1, (255,255,255), 2, cv2.LINE_AA)

        if Display:
            cv2.imshow('Distance', origImg)

    def calculateIntersects(self, rho, theta, width, height):
        a = np.cos(theta)
        b = np.sin(theta)

        points = []
        intersects = [None, None, None, None]

        if b != 0:
            # Check x = 0 crossing
            x1 = 0
            y1 = int(rho/b)
            if y1 >= 0 and y1 <= height:
                points.append((x1, y1))
                intersects[0] = 1

        if a != 0:
            # Check y = 0 crossing
            x1 = int(rho/a)
            y1 = 0
            if x1 >=0 and x1 <= width:
                points.append((x1, y1))
                intersects[1] = 1

        if b != 0:
            # Check x = width
            x1 = width
            y1 = int((rho - a*width)/b)
            if y1 >= 0 and y1 <= height:
                points.append((x1, y1))
                intersects[2] = 1

        if a != 0:
            # Check y = height
            x1 = int((rho - (b*height))/a)
            y1 = height
            if x1 >=0 and x1 <= width:
                points.append((x1, y1))
                intersects[3] = 1

        return points, intersects

    def findFrontGuide(self, origImg, frameTime, Display=True):
        # start = time.time()

        im = np.float32(origImg) / 255.0

        # Calculate gradient 
        gx = cv2.Sobel(im, cv2.CV_32F, 1, 0, ksize=1)
        gy = cv2.Sobel(im, cv2.CV_32F, 0, 1, ksize=1)

        mag, _ = cv2.cartToPolar(gx, gy, angleInDegrees=True)

        _, thresh = cv2.threshold(cv2.cvtColor(mag * 255, cv2.COLOR_BGR2GRAY), 10, 255, cv2.THRESH_BINARY)

        edges = cv2.Canny(np.uint8(thresh), 100, 200, apertureSize = 3)

        lines = cv2.HoughLines(edges, 1, np.pi/180, 210)

        height, width, _ = origImg.shape

        guideLine   = None
        guideTheta  = None

        halfPi      = math.pi/2.0

        if lines is not None:
            # print("Lines found: ", lines.shape)
            for line in lines:
                for rho,theta in line:

                    if theta < halfPi - 0.2 or theta > halfPi + 0.2:
                        break

                    points, _ = self.calculateIntersects(rho, theta, width, height)

                    if guideLine is None:
                        guideLine = points
                        guideTheta = theta
                    else:
                        if (points[0][1] + points[1][1]) > guideLine[0][1] + guideLine[1][1]:
                            guideLine = points
                            guideTheta = theta

                    #cv2.line(origImg, points[0], points[1], (0,0,255), 2)
        else:
            '''
            Line not found
            '''
            # print("No line detected")

        if guideLine is not None and Display:
            cv2.line(origImg, guideLine[0], guideLine[1], (0,255,0), 2)

        if Display:
            cv2.imshow('Front Guide Line', origImg)
            cv2.imshow('Edges', edges)
            cv2.imshow('Magnitude', mag)
        
        # end = time.time()

        # print(end - start)

        return guideLine, guideTheta
'''
Contains the MR. York and His vehicle detector
'''

import cv2

class Detector:

    def __init__(self, drone, vfov=45, hfov=80):
        self.drone              = drone
        self.vertical_fov       = vfov
        self.horizontal_fov     = hfov

    def findMrYork(self, origImg, frameTime, Display=True):
        
        if Display:
            cv2.imshow('Blue Channel', origImg[:,:,0])
            cv2.imshow('Green Channel', origImg[:,:,1])
            cv2.imshow('Red Channel', origImg[:,:,2])
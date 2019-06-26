#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from keyboard import KBHit

ON_GROUND   = "On Ground "
ON_AIR      = "On Air    "
TAKINGOFF   = "Taking off"
LANDING     = "Landing   "

class DroneController:

    def __init__(self):
        self.speed       = 0.2
        self.status      = ON_GROUND
        self.char        = ''
        self.kb          = KBHit()

        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.land_pub    = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        self.navi_pub    = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        rospy.init_node('drone__controller')
        self.rate = rospy.Rate(100)

    def print_help(self):
        print('Control:\nw\t: Forward\ns\t: Reverse\nd\t: Right\na\t: Left\ne\t: Yaw Right\nq\t: Yaw Left\nr\t: Ascend\nf\t: Descend\nSpace\t: Takeoff/land\n+\t: Increase Speed\n-\t: Decrease Speed')

    def print_status(self):
        print("Speed {0} Status {1}".format(self.speed, self.status))
        sys.stdout.write("\033[F") # Cursor up one line

    def adjust_speed(self):
        if self.char == '+':
            self.speed += 0.1
        if self.char == '-':
            self.speed -= 0.1
        
        if self.speed > 1.0:
            self.speed = 1.0
        if self.speed < 0.1:
            self.speed = 0.1

    def takeoff_landing(self):
        if self.char == ' ':
            if self.status == ON_GROUND:
                self.takeoff_pub.publish()
                self.status = TAKINGOFF
                self.print_status()
                rospy.sleep(2)
                self.kb.clear()
                self.status = ON_AIR
                
            elif self.status == ON_AIR:
                self.land_pub.publish()
                self.status = LANDING
                self.print_status()
                rospy.sleep(2)
                self.kb.clear()
                self.status = ON_GROUND
        
    def navigate(self):
        if self.status != ON_AIR:
            return
        
        cmd = Twist()
        
        if self.char == 'w':                # Forward
            cmd.linear.x    = self.speed
        elif self.char == 's':              # Reverse
            cmd.linear.x    = -1 * self.speed
        elif self.char == 'a':              # Left
            cmd.linear.y   = self.speed
        elif self.char == 'd':              # Right
            cmd.linear.y   = -1 * self.speed
        elif self.char == 'q':              # YAW Left
            cmd.angular.z   = self.speed
        elif self.char == 'e':              # YAW Right
            cmd.angular.z   = -1 * self.speed
        elif self.char == 'r':              # Ascend
            cmd.linear.z    = self.speed
        elif self.char == 'f':              # Descend
            cmd.linear.z    = -1 * self.speed   
        
        self.navi_pub.publish(cmd)

    def run(self):
        self.print_help()

        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.kb.kbhit():
                self.char = self.kb.getch()
                self.adjust_speed()
                self.takeoff_landing()
                self.navigate()
            self.print_status()
        
        self.kb.set_normal_term()

if __name__ == "__main__":
    try:
        dc = DroneController()
        dc.run()
    except rospy.ROSInterruptException:
        pass
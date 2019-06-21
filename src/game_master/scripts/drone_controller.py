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
        self.speed       = 0.3
        self.status      = ON_GROUND
        self.char        = ''

        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        self.land_pub    = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        self.navi_pub    = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        rospy.init_node('drone__controller')
        self.rate = rospy.Rate(100)

    def print_help(self):
        print('Control:\nw\t: Forward\ns\t: Reverse\nd\t: Turn Right\na\t: Turn Left\nSpace\t: Takeoff/land\n+\t: Increase Speed\n-\t: Decrease Speed')

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
                self.status = ON_AIR
            elif self.status == ON_AIR:
                self.land_pub.publish()
                self.status = LANDING
                self.print_status()
                rospy.sleep(2)
                self.status = ON_GROUND
        
    def navigate(self):
        if self.status != ON_AIR:
            return
        if self.char == 'w':
            cmd = Twist()
            cmd.linear.x    = self.speed
            cmd.linear.y    = 0
            cmd.linear.z    = 0
            cmd.angular.z   = 0
            self.navi_pub.publish(cmd)
        if self.char == 's':
            cmd = Twist()
            cmd.linear.x    = -1 * self.speed
            cmd.linear.y    = 0
            cmd.linear.z    = 0
            cmd.angular.z   = 0
            self.navi_pub.publish(cmd)
        if self.char == 'a':
            cmd = Twist()
            cmd.linear.x    = 0
            cmd.linear.y    = 0
            cmd.linear.z    = 0
            cmd.angular.z   = self.speed
            self.navi_pub.publish(cmd)
        if self.char == 'd':
            cmd = Twist()
            cmd.linear.x    = 0
            cmd.linear.y    = 0
            cmd.linear.z    = 0
            cmd.angular.z   = -1 * self.speed
            self.navi_pub.publish(cmd)

    def run(self):
        kb = KBHit()
        
        self.print_help()

        while not rospy.is_shutdown():
            self.rate.sleep()
            if kb.kbhit():
                self.char = kb.getch()
                self.adjust_speed()
                self.takeoff_landing()
                self.navigate()
            self.print_status()
        
        kb.set_normal_term()

if __name__ == "__main__":
    try:
        dc = DroneController()
        dc.run()
    except rospy.ROSInterruptException:
        pass
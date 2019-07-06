#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

class MessageRelay:

    def __init__(self):

        rospy.init_node('drone_msg_relay')

        self.takeoff_pub        = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.land_pub           = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        self.navi_pub           = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)

        self.takeoff_relay_sub  = rospy.Subscriber('/bebop_relay/takeoff', Empty, self.relay_takeoff)
        self.land_relay_sub     = rospy.Subscriber('/bebop_relay/land', Empty, self.relay_land)
        self.navi_relay_sub     = rospy.Subscriber('/bebop_relay/cmd_vel', Twist, self.relay_navi)

    def relay_takeoff(self, data):
        self.takeoff_pub.publish()

    def relay_land(self, data):
        self.land_pub.publish()

    def relay_navi(self, data):
        self.navi_pub.publish(data)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    MessageRelay().run()


#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

def drone_controller():
    takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
    land_pub    = rospy.Publisher('/bebop/land', Empty, queue_size=10)
    navi_pub    = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
    rospy.init_node('drone_controller')
    rate = rospy.Rate(1)
    counter = 0
    while not rospy.is_shutdown():
        counter += 1
        rate.sleep()
        if counter == 1:
            takeoff_pub.publish()
            rospy.loginfo("Taking off at count {0}".format(counter))
        elif counter > 3 and counter < 10:
            cmd = Twist()
            cmd.linear.x = 0.3
            cmd.linear.y = -0.3
            cmd.angular.z = 1
            navi_pub.publish(cmd)
        elif counter == 15:
            land_pub.publish()
            rospy.loginfo("Landing at count {0}".format(counter))
if __name__ == "__main__":
    try:
        drone_controller()
    except rospy.ROSInterruptException:
        pass
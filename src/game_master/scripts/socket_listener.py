#!/usr/bin/env python

import socket
import sys
import rospy
from std_msgs.msg import Empty, String, UInt8, Bool
from geometry_msgs.msg import Twist

class RosProxySocketListener:

    def __init__(self, port=50008):

        rospy.init_node('rosproxy_socket_listener')
        self.battery_status_pub     = rospy.Publisher('/drone/battery_status', UInt8, queue_size=1)
        self.target_pub             = rospy.Publisher('/drone/target', Twist, queue_size=1)
        self.intent_pub             = rospy.Publisher('/drone/intent', UInt8, queue_size=1)
        self.landrobot_visual_sub   = rospy.Subscriber('/landrobot/object_found', String, self.landrobot_found_object, queue_size=1)
        self.host                   = None # Bind with all interfaces
        self.port                   = port
        self.socket                 = None
        self.conn                   = None
        self.rate                   = rospy.Rate(100)
        self.buffer                 = ""
    
    def landrobot_found_object(self, data):
        if self.conn != None:
            try:
                self.conn.send("object_found,{0}".format(data.data) + "\n")
            except socket.error:
                print("Unable to pass object found message, connection error")
        else:
            print("Unable to pass object found message, not connected")

    def start(self):
        for res in socket.getaddrinfo(self.host, self.port, socket.AF_UNSPEC,
                              socket.SOCK_STREAM, 0, socket.AI_PASSIVE):
            af, socktype, proto, canonname, sa = res
            try:
                self.socket = socket.socket(af, socktype, proto)
            except socket.error as msg:
                self.socket = None
                continue
            try:
                self.socket.bind(sa)
                self.socket.listen(1)
            except socket.error as msg:
                self.socket.close()
                self.socket = None
                continue
            break
        if self.socket is None:
            print('could not open socket')
            sys.exit(1)

        print("Waiting for connection...")

        while not rospy.is_shutdown():
            self.conn, addr = self.socket.accept()
            print('Connected by {0}'.format(addr))
            # Deal with one connection at a time
            while not rospy.is_shutdown():
                data = self.conn.recv(64)
                if not data: break
                
                self.buffer += data
                lines = self.buffer.split('\n')
                self.buffer = lines[-1]

                for i in range(len(lines) - 1):

                    line = lines[i]
                    cmd = line.split(',')

                    if cmd[0] == "ping":
                        pass
                    elif cmd[0] == "battery_status":
                        self.battery_status_pub.publish(UInt8(cmd[1]))
                    elif cmd[0] == "intent":
                        self.intent_pub.publish(UInt8(cmd[1]))
                    elif cmd[0] == "target":
                        if len(cmd) != 3:
                            print("Invalid command: {0}".format(line))
                        target = Twist()
                        target.linear.x    = float(cmd[1])
                        target.angular.z   = float(cmd[2])
                        self.target_pub.publish(target)
                    else:
                        print("Unknown command: {0}".format(line))

                    self.conn.send("recieved" + "\n")
                    self.conn.flush()
                    self.rate.sleep()
            
            self.conn.close()
            self.conn = None


if __name__ == "__main__":
    try:
        RosProxySocketListener().start()
    except rospy.ROSInterruptException:
        pass
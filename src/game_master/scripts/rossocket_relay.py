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
        self.bear_direction_pub     = rospy.Publisher('/drone/bear_direction', String, queue_size=1)
        self.landrobot_visual_sub   = rospy.Subscriber('/landrobot/object_found', String, self.landrobot_found_object, queue_size=1)
        self.control_sub            = rospy.Subscriber('/drone/go', Empty, self.lets_play, queue_size=1)
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

    def lets_play(self, data):
        if self.conn != None:
            try:
                self.conn.send("go\n")
            except socket.error:
                print("Unable to pass go message, connection error")
        else:
            print("Unable to pass go message, not connected")

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
            self.conn.setblocking(0)
            print('Connected by {0}'.format(addr))
            # Deal with one connection at a time
            while not rospy.is_shutdown():
                data = None
            
                try:
                    data = self.conn.recv(1)
                except socket.timeout:
                    self.rate.sleep()
                except socket.error as ex:
                    if ex.errno == 11:
                        self.rate.sleep()
                    else:
                        print(ex)
                        break
                except Exception:
                    self.conn.close()
                    break

                if not data: continue
                
                if data == '\n':
                    cmd = self.buffer.split(',')

                    if cmd[0] == "battery_status":
                        print(cmd)
                        print(cmd[0])
                        print(cmd[1])
                        self.battery_status_pub.publish(UInt8(int(cmd[1])))
                    elif cmd[0] == "intent":
                        self.intent_pub.publish(UInt8(int(cmd[1])))
                    elif cmd[0] == "bear_direction":
                        self.intent_pub.publish(String(cmd[1]))
                    elif cmd[0] == "target":
                        if len(cmd) != 4:
                            print("Invalid command: {0}".format(self.buffer))
                        target = Twist()
                        target.linear.x     = float(cmd[1])
                        target.linear.y     = float(cmd[2])
                        target.angular.z    = float(cmd[3])
                        self.target_pub.publish(target)
                    else:
                        print("Unknown command: {0}".format(self.buffer))

                    self.buffer = ''

                    self.conn.send("recieved" + "\n")
                else:
                    self.buffer += data
            
            self.conn.close()
            self.conn = None


if __name__ == "__main__":
    try:
        RosProxySocketListener().start()
    except rospy.ROSInterruptException:
        pass
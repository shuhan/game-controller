#!/usr/bin/env python

import socket
import sys
import rospy
from std_msgs.msg import Empty, String, UInt8, Bool
from geometry_msgs.msg import Twist

class RosProxySocketClient:

    def __init__(self, host='144.32.145.223', port=50008):

        rospy.init_node('rosproxy_socket_client')

        self.host                   = host
        self.port                   = port
        self.socket                 = None
        self.rate                   = rospy.Rate(100)
        self.battery_status_sub     = rospy.Subscriber('/drone/battery_status', UInt8, self.battery_status_received, queue_size=1)
        self.target_sub             = rospy.Subscriber('/drone/target', Twist, self.target_received, queue_size=1)
        self.intent_sub             = rospy.Subscriber('/drone/intent', UInt8, self.intent_received, queue_size=1)
        self.landrobot_visual_pub   = rospy.Publisher('/landrobot/object_found', String, queue_size=1)
        self.buffer                 = ""

    def battery_status_received(self, data):
        if self.socket != None:
            try:
                cmd = "battery_status,{0}".format(data.data)
                print(cmd)
                self.socket.send(cmd + "\n")
            except socket.error:
                print("Unable to pass battery status, connection error")
        else:
            print("Unable to pass battery status, not connected")

    def target_received(self, data):
        if self.socket != None:
            try:
                cmd = "target,{0:.2f},{1:.6f}".format(data.linear.x, data.angular.z)
                print(cmd)
                self.socket.send(cmd + "\n")
            except socket.error:
                print("Unable to pass target, connection error")
        else:
            print("Unable to pass target, not connected")

    def intent_received(self, data):
        if self.socket != None:
            try:
                cmd = "intent,{0}".format(data.data)
                print(cmd)
                self.socket.send(cmd + "\n")
            except socket.error:
                print("Unable to pass intent, connection error")
        else:
            print("Unable to pass intent, not connected")

    def connect(self):
        for res in socket.getaddrinfo(self.host, self.port, socket.AF_UNSPEC, socket.SOCK_STREAM):
            af, socktype, proto, _, sa = res
            try:
                self.socket = socket.socket(af, socktype, proto)
            except socket.error as _:
                self.socket = None
                continue
            try:
                self.socket.connect(sa)
            except socket.error as _:
                self.socket.close()
                self.socket = None
                continue
            break
        if self.socket is None:
            print 'could not open socket'
            sys.exit(1)

        self.socket.setblocking(0)
        
        while not rospy.is_shutdown():
            data = None

            try:
                data = self.socket.recv(1)
            except socket.timeout:
                self.rate.sleep()
            except socket.error as ex:
                if ex.errno == 11:
                    self.rate.sleep()
                else:
                    print(ex)
                    break
            except Exception:
                break

            if not data: continue
                
            if data == '\n':

                cmd = self.buffer.split(',')

                if cmd[0] == "recieved":
                    pass
                elif cmd[0] == "object_found":
                    self.landrobot_visual_pub.publish(String(cmd[1]))
                else:
                    print("Unknown command: {0}".format(self.buffer))

                self.buffer = ''

            else:
                self.buffer += data
        
        self.socket.close()

if __name__ == "__main__":
    try:
        RosProxySocketClient().connect()
    except rospy.ROSInterruptException:
        pass
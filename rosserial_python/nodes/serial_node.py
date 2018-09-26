#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__author__ = "mferguson@willowgarage.com (Michael Ferguson)"

import rospy
from rosserial_python import SerialClient, RosSerialServer, TcpClient
from serial import SerialException
from time import sleep
import multiprocessing

import sys

if __name__=="__main__":

    rospy.init_node("serial_node")
    rospy.loginfo("ROS Serial Python Node")

    port_name = rospy.get_param('~port','/dev/ttyUSB0')
    baud = int(rospy.get_param('~baud','57600'))
    tcp_portnum = int(rospy.get_param('~tcp_port', '6000'))

    # Use TCP Port
    if port_name == "tcp":

        while not rospy.is_shutdown():
            rospy.loginfo("Connecting to %s at port %d" % ('localhost', tcp_portnum) )
            try:
                rospy.loginfo("Recreating tcp client.")
                tcp_client = TcpClient('localhost', tcp_portnum)
                client = SerialClient(tcp_client)
                client.run()
            except KeyboardInterrupt:
                break
            except socket.error:
                sleep(0.1)
                continue
            except OSError:
                sleep(0.1)
                continue

    # Use Serial Port
    else:
        while not rospy.is_shutdown():
            rospy.loginfo("Connecting to %s at %d baud" % (port_name,baud) )
            try:
                rospy.loginfo("Recreating serial client.")
                client = SerialClient(port_name, baud)
                client.run()
            except KeyboardInterrupt:
                break
            except SerialException:
                sleep(0.1)
                continue
            except OSError:
                sleep(0.1)
                continue

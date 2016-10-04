#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
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
# 
# Modified by Bhavya Gupta for using config files

THIS_PACKAGE = "rosserial_arduino"

__usage__ = """
make_libraries_isolated.py generates the Arduino rosserial library files for a
set of packages and installs them as per the directory structure provided in 
a yaml config file.

rosrun rosserial_arduino make_libraries.py <yaml config file>
"""

import rospkg
import rosserial_client
from rosserial_client.make_library import *
import yaml
import os

# for copying files
import shutil

ROS_TO_EMBEDDED_TYPES = {
    'bool'    :   ('bool',              1, PrimitiveDataType, []),
    'byte'    :   ('int8_t',            1, PrimitiveDataType, []),
    'int8'    :   ('int8_t',            1, PrimitiveDataType, []),
    'char'    :   ('uint8_t',           1, PrimitiveDataType, []),
    'uint8'   :   ('uint8_t',           1, PrimitiveDataType, []),
    'int16'   :   ('int16_t',           2, PrimitiveDataType, []),
    'uint16'  :   ('uint16_t',          2, PrimitiveDataType, []),
    'int32'   :   ('int32_t',           4, PrimitiveDataType, []),
    'uint32'  :   ('uint32_t',          4, PrimitiveDataType, []),
    'int64'   :   ('int64_t',           8, PrimitiveDataType, []),
    'uint64'  :   ('uint64_t',          4, PrimitiveDataType, []),
    'float32' :   ('float',             4, PrimitiveDataType, []),
    'float64' :   ('float',             4, AVR_Float64DataType, []),
    'time'    :   ('ros::Time',         8, TimeDataType, ['ros/time']),
    'duration':   ('ros::Duration',     8, TimeDataType, ['ros/duration']),
    'string'  :   ('char*',             0, StringDataType, []),
    'Header'  :   ('std_msgs::Header',  0, MessageDataType, ['std_msgs/Header'])
}

# need correct inputs
if (len(sys.argv) < 2):
    print __usage__
    exit()
    
# get config file name
config_file = sys.argv[1]
fd_in = open(config_file)
contents = fd_in.read()
fd_in.close()
parsed_contents = yaml.load(contents)

microcontroller_ws = parsed_contents["microcontroller_ws"]
header_out = os.path.join(microcontroller_ws, parsed_contents["header_output_directory"])
source_out = os.path.join(microcontroller_ws, parsed_contents["source_output_directory"])
custom_ws = parsed_contents["catkin_workspaces"]
packages = parsed_contents["packages"]

print ("\033[40;33mMicrocontroller workspace:\033[0m %s " % microcontroller_ws)
print ("\033[40;33mHeader output            :\033[0m %s " % header_out)
print ("\033[40;33mSource output            :\033[0m %s " % source_out)
print ("\033[40;33mCatkin workspaces        :\033[0m %s " % custom_ws)
print ("\033[40;33mPackages                 :\033[0m %s " % packages)
print ("----------------------------------------------------------------------")

# "Overlay" the custom catkin workspaces
ros_package_path = os.environ["ROS_PACKAGE_PATH"]
for ws in custom_ws:
    ros_package_path = ros_package_path + ":" + ws
os.environ["ROS_PACKAGE_PATH"] = ros_package_path
# TODO: Check if the messages are built?
rospack = rospkg.RosPack()

for pkg in packages:
  rosserial_generate_package(rospack, pkg, header_out, ROS_TO_EMBEDDED_TYPES)

# Copy the non-message files:
header_ros_out = os.path.join(header_out, "ros")
header_tf_out = os.path.join(header_out, "tf")
if not os.path.exists(header_ros_out):
  os.makedirs(header_ros_out)
if not os.path.exists(header_tf_out):
  os.makedirs(header_tf_out)
if not os.path.exists(source_out):
  os.makedirs(source_out)
source_files = ["duration.cpp", "time.cpp"]
base_header_files = ['ros/duration.h',
                     'ros/msg.h',
                     'ros/node_handle.h',
                     'ros/publisher.h',
                     'ros/service_client.h',
                     'ros/service_server.h',
                     'ros/subscriber.h',
                     'ros/time.h',
                     'tf/tf.h',
                     'tf/transform_broadcaster.h']
arduino_header_files = ['ArduinoHardware.h', 'ros.h']

rosserial_client_dir = rospack.get_path("rosserial_client")
rosserial_arduino_dir = rospack.get_path("rosserial_arduino")
for f in source_files:
  shutil.copy(os.path.join(rosserial_client_dir, "src/ros_lib/", f), os.path.join(source_out, f));

for f in base_header_files:
  shutil.copy(os.path.join(rosserial_client_dir, "src/ros_lib/", f), os.path.join(header_out, f));

for f in arduino_header_files:
  shutil.copy(os.path.join(rosserial_arduino_dir, "src/ros_lib/", f), os.path.join(header_out, f));

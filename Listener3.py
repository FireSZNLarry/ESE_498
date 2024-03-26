#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
import time

# status is a global that I'll use to print
# 0 - imu   1 - 5603    2 - both    3 - neither
# While I'm not a fan of globals, in this case, you can use 
# globals to pass information from one of the call functions to the other
status = 0

def call_key(data):
    rospy.loginfo(rospy.get_caller_id() + 'Keyboard %s', data.data)
    print("keyboard: " + str(data.data))

    global status
    if (isinstance(int(str(data.data)), int)):
        status = int(str(data.data))
        if (status > 4):
            status = 0

def call_imu(data):
    # rospy.loginfo(rospy.get_caller_id() + 'imu %s', data.data)
    [ax, ay, az] = str(data.data).split()
    ax = float(ax)  # destringify
    ay = float(ay)
    az = float(az)
    global status
    if (status == 0 or status == 2):
        #print('imu: ', data.data)
        print(f'ax: {ax:.2f}\tmy: {ay:.2f}\taz: {az:.2f}')

def call_mmc5603(data):
    # rospy.loginfo(rospy.get_caller_id() + 'mmc5603 %s', data.data)
    [mx, my, mz, temp] = str(data.data).split()
    mx = float(mx)
    my = float(my)
    mz = float(mz)
    temp = float(temp)
    global status
    if (status == 1 or status == 2):
        #print('mms5603: ', data.data)
        print(f'mx: {mx:.2f}\tmy: {my:.2f}\tmz: {mz:.2f}\ttemp: {temp:.1f}')

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('keyboard', String, call_key)
    rospy.Subscriber('mmc5603',   String, call_mmc5603)
    rospy.Subscriber('imu',   String, call_imu)

    rospy.spin()

if __name__ == '__main__':
    listener()

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

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import Int32MultiArray
import cv2 as cv
from realsense_camera import *
from mask_rcnn import *

rs= RealsenseCamera()
mrcnn=MaskRCNN()

def ImageCapture():
    #This is my issues 
    #So depth Frame Values is a 2D Matrix 720, 1280
    #So BGR   Frame Values is a 3D Matrix thing 720 by 1280 by 3
    
    pub = rospy.Publisher('chatter', Int32MultiArray, queue_size=10)
    rospy.init_node('ImageCapture', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
    #while True:
    #Get info in real time
    	
        ret, bgr_frame, depth_frame = rs.get_frame_stream()
        boxes, classes, contours, centers = mrcnn.detect_objects_mask(bgr_frame)
        mrcnn.draw_object_info(bgr_frame,depth_frame)
        bgr_frame = mrcnn.draw_object_mask(bgr_frame)
        rospy.loginfo(bgr_frame)
        pub.publish(bgr_frame)
        rate.sleep()


if __name__ == '__main__':
    try:
        ImageCapture()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
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
import numpy as np
from std_msgs.msg import String
import cv2

pub = rospy.Publisher('chatter', String, queue_size=10)
old_points = np.array([[]])
def select_point(event, x, y, flags, params):
	global pointTL,pointBR, point_selected, old_points
	if event == cv2.EVENT_LBUTTONDOWN:
		pointTL = (x, y)
	if event == cv2.EVENT_LBUTTONUP:
		pointBR = (x,y)
		point_selected = True
		old_points = np.array([list(pointTL),list(pointBR)], dtype=np.float32)
		print(old_points)
class classfication():
	def __init__(self):
		
		cv2.namedWindow("Frame")
		cv2.setMouseCallback("Frame", select_point)
	def trial(self):
		cap = cv2.VideoCapture("Plate4Colour.mp4")
		_, frame = cap.read()
		cv2.imshow("Frame", frame)
		cv2.waitKey(0)
		pub.publish("5")
		global old_points
		lk_params = dict(winSize = (15, 15),
				maxLevel = 4,
				criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
		old_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		while True:
			_, frame = cap.read()
			gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			if point_selected is True:
				#cv2.circle(frame, point, 5, (0, 0, 255), 2)
				cv2.rectangle(frame,pointTL,pointBR,(0,0,255),2)
				new_points, status, error = cv2.calcOpticalFlowPyrLK(old_gray, gray_frame, old_points, None, **lk_params)
				old_gray = gray_frame.copy()
				old_points = new_points
				pointTL1 = tuple(new_points[0])
				pointBR1 = tuple(new_points[1])
				pub.publish(np.array_str(old_points,precision=4,suppress_small=True))
				cv2.rectangle(frame,pointTL1,pointBR1,(0,255,0),2)
			cv2.imshow("Frame", frame)
			

			key = cv2.waitKey(60)
			if(key == ord("b")):
				break
		return 
def talker():
 
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    if not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
	test = classfication()
	test.trial()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

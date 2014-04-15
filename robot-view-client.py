#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2013, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */

import diff_drive
import hubo_ach
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np

dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

HUBO_CHAN_VIEW   = 'robot-vid-chan'
ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_TIME_CHAN  = 'robot-time'

# CV setup 
cv.NamedWindow("w2", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240

nx = 640
ny = 480

curr_err = 0
prev_err = 0
err_sum = 0

err_arr = [0,0,0,0,0]

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()

t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

v = ach.Channel(HUBO_CHAN_VIEW)
v.flush()
i=0

print '======================================'
print '============= Hubo-View =============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'

# Open file for error data
f = open("err_data.txt", "w")

while True:
    # Get Frame
#    f, img = capture.read()
    img = np.zeros((newx,newy,3), np.uint8)
    #gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_image = img.copy()
    vid = cv2.resize(gray_image,(newx,newy))

    [status, framesize] = v.get(vid, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vid,(nx,ny))
# Change image back to blue because it was showing as red
	vid2 = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("w2", vid2)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )

#    print vid

    i=i+1
    print i
    time.sleep(0.1)

#--------------------------------------------------------------------
# Get start time of the sim
    otime2=tim.sim[0]

# Create a frame for finding blue colors
    blue = cv2.inRange(vid2, np.array([0,0,0], dtype = np.uint8), np.array([255,0,0], dtype = np.uint8));

# Finds the blue box
    cntRGB, h = cv2.findContours(blue, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

# Finds the center of the box and calculates its center
    for cnt in cntRGB:
	(x, y), radius = cv2.minEnclosingCircle(cnt)
	center = (int(x), int(y))
	radius = int(radius)
	check=1
	print 'x, y ', x, y

# Get the error of the robot center from the box center
    err = (nx/2) - x;
    print 'Error (pixels) = ',err
    err_per = abs(err/newx);
    print 'Error (%) = ',err_per*100,' %'

# Integral Control
    for num in range(0,3):
	err_arr[num] = err_arr[num+1]
	err_sum = err_sum + err_arr[num]

    err_arr[4] = err
    err_sum = err_sum + err_arr[4]

# Porportional and Derivative Control with summation
    curr_err = err
    u_in = 2*err + 2*(curr_err-prev_err) + 0.2*err_sum
    prev_err = curr_err

# Input used to set motor speed
    u_in_per = abs(u_in/newx);
    print'Motor Vel. input(%) = ',u_in_per*100,'%'
    
# Motor speed inputs determining spin direction
    if(u_in > 0.5):
	ref.ref[0] = 0.5*u_in_per
	ref.ref[1] = -0.5*u_in_per
    elif(u_in < -0.5):
	ref.ref[0] = -0.5*u_in_per
	ref.ref[1] = 0.5*u_in_per
    else:
	ref.ref[0] = 0
	ref.ref[1] = 0

    r.put(ref);

# Write error data to file
    f.write(str(err))
    f.write('\n') 

# Have the simulation wait
    while((tim.sim[0]-otime2)<.1):
	[status, framesize] = t.get(tim, wait=False, last=True)

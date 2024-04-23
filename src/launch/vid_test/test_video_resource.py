#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""

Copyright (c) 2015 PAL Robotics SL.
Released under the BSD License.

Created on 7/14/15

@author: Sammy Pfeiffer

test_video_resource.py contains
a testing code to see if OpenCV can open a video stream
useful to debug if video_stream does not work
"""

import cv2
import sys

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("You must give an argument to open a video stream.")
        print("  It can be a number as video device, e.g.: 0 would be /dev/video0")
        print("  It can be a URL of a stream,        e.g.: rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov")
        print("  It can be a video file,             e.g.: myvideo.mkv")
        sys.exit(0)

    resource = sys.argv[1]
    # If we are given just a number, interpret it as a video device
    if len(resource) < 3:
        resource_name = "/dev/video" + resource
        resource = int(resource)
    else:
        resource_name = resource
    print("Trying to open resource: " + resource_name)
    cap = cv2.VideoCapture(resource)
    if not cap.isOpened():
        print("Error opening resource: " + str(resource))
        print("Maybe OpenCV VideoCapture can't open it")
        sys.exit(0)

    print("Correctly opened resource, starting to show feed.")
    rval, frame = cap.read()
    while rval:
        cv2.imshow("Stream: " + resource_name, frame)
        rval, frame = cap.read()
        key = cv2.waitKey(20)
        # print("key pressed: " + str(key))
        # Exit on ESC, you may want to uncomment the print to know which key is ESC for you
        if key == 27 or key == 1048603:
            break
    cv2.destroyWindow("preview")

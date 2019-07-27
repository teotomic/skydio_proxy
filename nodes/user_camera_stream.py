"""
Remote Control Demo

Use this script to control the vehicle from a separate computer over WiFi, no phone required.

You can send movement commands via a connected USB gamepad or your keyboard.

This script uses an RTP stream from the vehicle to create an opencv window.
"""
# prep for python 3.0
from __future__ import absolute_import
from __future__ import print_function
import argparse
import os
import threading
import time

# Include rtp when importing opencv so we can process the video stream from the vehicle.
os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = "protocol_whitelist;file,rtp,udp"
import cv2  # pylint: disable=import-error
QUIT = ord('q')

def main():
    stream_file = 'h264_stream.sdp'

    # Create an opencv video input source from the RTP stream description file.
    cap = cv2.VideoCapture(stream_file)

    while True:
        # Get a frame and show it.
        _, frame = cap.read()
        if frame is not None:
            cv2.imshow('skydio', frame)
        key = cv2.waitKey(66)

        # TODO: stream to ROS

        # Quit the program if you press Q
        if key & 0xff == QUIT:
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

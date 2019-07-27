#!/bin/bash
# Opens a gstreamer window with the R1 user camera stream
gst-launch-1.0 udpsrc port=55004 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96' ! rtpjitterbuffer ! rtph264depay ! decodebin ! videoconvert ! autovideosink

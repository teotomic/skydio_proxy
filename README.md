# Skydio ROS proxy

## Getting started

First, get access to the Skydio SDK and follow steps as described [here](https://github.com/Skydio/skydio-skills).
You will need to upload the contents of the folder *skillset* to your skydio skillset.
This package contains a slightly modified version of the skydio UDP link code with support for the
voxel map and other channels.

The package contains a simple "industrial inspection" skill that flies to predefined camera poses
defined by waypoints and gimbal angles, and lands at the end. It streams the vehicle voxel map,
pose, gimbal pose, and the downsampled 15Hz user camera stream.

## ROS Proxy
To start the proxy, call

    rosrun skydio_proxy nodes/udp_proxy.py

This will connect to the vehicle and expose the /skydio_command service. In order to start a mission,
you will need to setup your user email and skillset name.
In order to go to PREP mode and start streaming video and messages, run

    rosservice call /skydio_command prep

Note that this will heat up the vehicle significantly if not flying. It is recommended to cool the
R1 with an external fan while in this mode.

To take off or land:

    rosservice call /skydio_command takeoff
    rosservice call /skydio_command land

To switch to inspection.industrial_inspection skill:

    rosservice call /skydio_command start_mission

All of this can be done from the simple *service_gui.py* node, which just calls the corresponding
services on button presses. To start just the user camera stream to ROS:

     roslaunch skydio_proxy launch/camera_stream.launch

The voxel republisher node translates RLE-encoded voxel map coming from the drone and publishes it
as a point cloud. It is written in C++, so you have to compile the package first. It is very
rudimentary, and only publishes the current voxel map, along with a filtered version. There is a lot
of room for improvement for this node, so feel free to fork and make a pull request.
To start the voxel republisher node, run

     rosrun skydio_proxy voxel_republisher_node

The launch file *proxy.launch* will start the udp proxy, voxel republisher, user camera stream, and
the service GUI. Make sure to setup your user email and skillset name first.

## Webcam
Packages:

    sudo apt-get install gstreamer1.0 gstreamer1.0-plugins-good gstreamer1.0-ffmpeg gstreamer1.0-plugins-ugly 

Start webcam stream to RTP:

    gst-launch-1.0 v4l2src device="/dev/video0" ! image/jpeg,width=1280,height=720,framerate=60/1 ! jpegdec ! videoconvert ! x264enc pass=qual quantizer=5 tune=zerolatency ! rtph264pay ! udpsink host=127.0.0.1 port=5200

Display RTP stream:

    gst-launch-1.0 udpsrc port=5200 ! "application/x-rtp, payload=127" ! rtph264depay ! avdec_h264 ! xvimagesink sync=false


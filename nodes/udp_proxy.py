#!/usr/bin/env python
"""
ROS proxy for Skydio vehicles.
"""
# prep for python 3.0
from __future__ import absolute_import
from __future__ import print_function
import argparse
import os
import subprocess
import sys
import threading
import time
import math
import json

pkg_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, pkg_path)
from skydio.comms.http_client import HTTPClient
from skydio.comms.udp_link import UDPLink
from skydio.lcmtypes.common import *
from skydio.lcmtypes.body import *
from skydio.lcmtypes.voxels import *
from skydio.types import vehicle_pb2
from util import channel_util

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from skydio_proxy.msg import VoxelOccupancyRunLengthEncoded
from skydio_proxy.msg import SkillStatus
from skydio_proxy.srv import SkydioCommand

# NOTE(teo): this is the name of the skill that will be used then starting a mission. Be sure to
# store it under the correct skillset, and check your manifest.json.
SKILLSET_NAME = 'msrm'
INSPECTION_SKILL = '{}.inspection.IndustrialInspection'.format(SKILLSET_NAME)

class SkydioUdpProxy:
    def __init__(self, udp_link=None, http_client=None):
        self.link = udp_link
        self.client = http_client

        # Publishers
        self.imu_pub = rospy.Publisher('~imu', Imu, queue_size=1)
        self.pose_pub = rospy.Publisher('~pose', PoseStamped, queue_size=1)
        self.gimbal_pose_pub = rospy.Publisher('~gimbal_pose', PoseStamped, queue_size=1)
        self.voxel_pub = rospy.Publisher('~voxel_rle', VoxelOccupancyRunLengthEncoded, queue_size=1)
        self.status_pub = rospy.Publisher('~skill_status', SkillStatus, queue_size=1)

        self.imu_msg = Imu()
        self.pose_msg = PoseStamped()
        self.gimbal_pose_msg = PoseStamped()
        self.voxel_msg = VoxelOccupancyRunLengthEncoded()

        # Sequence numbers
        self.imu_seq = 0
        self.pose_seq = 0
        self.gimbal_pose_seq = 0
        self.voxel_seq = 0
        self.status_seq = 0

        # TF frames
        self.vehicle_frame = rospy.get_param('~vehicle_frame', 'vehicle')
        self.voxel_frame = rospy.get_param('~voxel_frame', 'voxel')
        self.nav_frame = rospy.get_param('~nav_frame', 'nav')
        self.gimbal_frame = rospy.get_param('~gimbal_frame', 'gimbal')
        self.world_frame = rospy.get_param('~world_frame', 'world')

        self.br = tf.TransformBroadcaster()
        self.skill_status = None

    def set_skill_status(self, status):
        self.skill_status = status

    def run(self):
        rospy.loginfo('Streaming...')
        while not rospy.is_shutdown():
            msg = self.link.read()

            if isinstance(msg, imu_meas_t):
                self.publish_imu(msg)
                continue

            if isinstance(msg, pose_state_t):
                self.publish_pose(msg)
                self.publish_tf(msg)
                continue

            if isinstance(msg, pose_state_t):
                self.publish_pose(msg)
                self.publish_tf(msg)
                continue

            if isinstance(msg, vehicle_pb2.GimbalNavTransform):
                self.publish_gimbal_pose(msg.nav_T_gimbal_camera_imu)
                continue

            if isinstance(msg, vehicle_pb2.VoxelOccupancyRunLengthEncoded):
                #self.publish_voxel(VoxelMap.from_pb(msg)
                self.publish_voxels(msg)
                continue

    def publish_imu(self, lcm_msg):
        msg = self.imu_msg

        msg.linear_acceleration.x = lcm_msg.acceleration.data[0]
        msg.linear_acceleration.y = lcm_msg.acceleration.data[1]
        msg.linear_acceleration.z = lcm_msg.acceleration.data[2]

        msg.angular_velocity.x = lcm_msg.angular_velocity.data[0]
        msg.angular_velocity.y = lcm_msg.angular_velocity.data[1]
        msg.angular_velocity.z = lcm_msg.angular_velocity.data[2]

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.vehicle_frame
        msg.header.seq = self.imu_seq
        self.imu_seq += 1

        self.imu_pub.publish(msg)

    def fill_pose_msg(self, msg, lcm_msg):
        msg.pose.position.x = lcm_msg.position.data[0]
        msg.pose.position.y = lcm_msg.position.data[1]
        msg.pose.position.z = lcm_msg.position.data[2]

        self.imu_msg.orientation.x = lcm_msg.orientation.xyzw[0]
        self.imu_msg.orientation.y = lcm_msg.orientation.xyzw[1]
        self.imu_msg.orientation.z = lcm_msg.orientation.xyzw[2]
        self.imu_msg.orientation.w = lcm_msg.orientation.xyzw[3]

        msg.pose.orientation.x = lcm_msg.orientation.xyzw[0]
        msg.pose.orientation.y = lcm_msg.orientation.xyzw[1]
        msg.pose.orientation.z = lcm_msg.orientation.xyzw[2]
        msg.pose.orientation.w = lcm_msg.orientation.xyzw[3]

        msg.header.stamp = rospy.Time.now()

        return msg

    def publish_pose(self, lcm_msg):
        msg = self.fill_pose_msg(self.pose_msg, lcm_msg)
        msg.header.frame_id = self.nav_frame
        msg.header.seq = self.pose_seq
        self.pose_seq += 1

        self.pose_pub.publish(msg)
        self.publish_pose_tf(lcm_msg, self.vehicle_frame)

    def publish_gimbal_pose(self, lcm_msg):
        msg = self.fill_pose_msg(self.gimbal_pose_msg, lcm_msg)
        msg.header.frame_id = self.nav_frame
        msg.header.seq = self.gimbal_pose_seq
        self.gimbal_pose_seq += 1

        self.gimbal_pose_pub.publish(msg)
        self.publish_pose_tf(lcm_msg, self.gimbal_frame)

    def publish_pose_tf(self, lcm_msg, target_frame):
        curr_time = rospy.Time.now()

        # Publish nav transform
        self.br.sendTransform((lcm_msg.position.data[0], lcm_msg.position.data[1],
                               lcm_msg.position.data[2]),
                              (lcm_msg.orientation.xyzw[0], lcm_msg.orientation.xyzw[1],
                               lcm_msg.orientation.xyzw[2], lcm_msg.orientation.xyzw[3]),
                              curr_time,
                              target_frame,
                              self.nav_frame)

    def publish_tf(self, lcm_msg):
        curr_time = rospy.Time.now()
        # Publish nav transform
        self.br.sendTransform((0.0, 0.0, 0.0),
                              (0.0, 0.0, 0.0, 1.0),
                              curr_time,
                              self.nav_frame,
                              self.world_frame)

        # Publish voxel transform -- at drone position, but axis aligned and rounded to the
        # nearest integer
        self.br.sendTransform((lcm_msg.position.data[0], lcm_msg.position.data[1],
                               lcm_msg.position.data[2]),
                              (0, 0, 0, 1),
                              curr_time,
                              self.voxel_frame,
                              self.nav_frame)

    def publish_voxels(self, voxels):
        msg = self.voxel_msg

        msg.voxel_header.scale = voxels.header.scale
        msg.voxel_header.dims[0] = voxels.header.dims.data[0]
        msg.voxel_header.dims[1] = voxels.header.dims.data[1]
        msg.voxel_header.dims[2] = voxels.header.dims.data[2]
        msg.voxel_header.origin[0] = voxels.header.origin.data[0]
        msg.voxel_header.origin[1] = voxels.header.origin.data[1]
        msg.voxel_header.origin[2] = voxels.header.origin.data[2]

        msg.runs = voxels.runs

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.nav_frame
        msg.header.seq = self.voxel_seq
        self.voxel_seq += 1

        self.voxel_pub.publish(msg)

    def publish_skill_status(self, skill_key, status_json):
        msg = SkillStatus()
        msg.skill_key = skill_key
        msg.status_json_encoded = status_json

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = ''
        msg.header.seq = self.status_seq
        self.status_seq += 1

        self.status_pub.publish(msg)


class SkillStatusThread(threading.Thread):
    """
    A thread that periodically polls the selected skill and publishes the JSON-encoded
    response to a ROS topic.
    """
    def __init__(self, http_client, ros_proxy):
        threading.Thread.__init__(self)
        self.http_client = http_client
        self.ros_proxy = ros_proxy

        # Params
        self.land_on_stop = rospy.get_param('~land_on_skill_finished', True)
        self.skill_poll_period = float(rospy.get_param('~skill_poll_period', 1.0))


    def handle_skill_status(self, skill_status):
        rospy.loginfo('Skill status: {}'.format(skill_status))

        if 'state' in skill_status:
            if skill_status['state'].lower() == 'finished' and self.land_on_stop:
                rospy.loginfo('Skill is finished, landing')
                self.http_client.land()

    def run(self):
        if self.skill_poll_period < 1e-6:
            rospy.logwarn("Skill poll period set to zero, not starting polling thread")
            return

        rospy.loginfo("Started skill status thread, will {}land when skill is stopped".
                      format('' if self.land_on_stop else 'not '))

        while not rospy.is_shutdown():
            # Get active skill from the HTTP client. Has to be set by a call to
            # HttpClient.set_skill().
            active_skill = self.http_client.get_active_skill()
            if not active_skill:
                time.sleep(self.skill_poll_period)
                continue

            # TODO(teo): use udp link -- send_rpc_data()
            response = self.http_client.send_custom_comms(active_skill, "")

            # Call skill status handler
            if response:
                if 'data' in response:
                    self.ros_proxy.publish_skill_status(active_skill, response['data'])
                    try:
                        skill_status = json.loads(response['data'])
                        self.ros_proxy.set_skill_status(response['data'])
                        self.handle_skill_status(skill_status)
                    except ValueError, e:
                        print(e)

            time.sleep(self.skill_poll_period)


class PilotStatusThread(threading.Thread):
    """
    A thread that periodically updates the PILOT status.
    """
    def __init__(self, http_client):
        threading.Thread.__init__(self)
        self.http_client = http_client

    def run(self):
        last_flight_phase = None
        while not rospy.is_shutdown():
            self.http_client.update_pilot_status()

            # Print flight phase on change
            current_flight_phase = self.http_client.get_last_flight_phase()
            if current_flight_phase != last_flight_phase:
                rospy.loginfo('Flight phase: {}'.format(current_flight_phase))
            last_flight_phase = current_flight_phase

            time.sleep(2)


def start_skill_thread(client, proxy):
    """
    Start a thread to periodically poll the skill status via RPC call.

    Args:
        client (skydio.comms.HttpClient): Connected HTTP client instance
        proxy (SkydioUdpProxy): ROS UDP Proxy instance
    """
    skill_thread = SkillStatusThread(client, proxy)
    skill_thread.setDaemon(True)
    skill_thread.start()

    return skill_thread


def start_pilot_update_thread(client):
    """
    Start a thread to periodically poll the status endpoint to keep ourselves the active pilot.

    Args:
        client (skydio.comms.HttpClient): Connected HTTP client instance
    """
    status_thread = PilotStatusThread(client)
    status_thread.setDaemon(True)
    status_thread.start()

    return status_thread


def main():
    rospy.init_node('skydio_proxy', anonymous=False)

    # Port configuration
    h264_stream_port = int(rospy.get_param('~h264_stream_port', 55004))
    udp_port = int(rospy.get_param('~udp_port', 50112))

    # H264 is the 720P 15fps h264 encoded stream directly from the camera.
    stream_settings = {'source': 'H264', 'port': h264_stream_port}

    # Create the client to use for all requests.
    baseurl = rospy.get_param('~baseurl', 'http://192.168.10.1')
    token_file = rospy.get_param('~token_file', None)
    client = HTTPClient(baseurl,
                        pilot=True,
                        token_file=token_file,
                        stream_settings=stream_settings)

    if not client.check_min_api_version():
        print('Your vehicle is running an older api version.'
                      ' Update recommended in order to enable streaming.')

    # Periodically poll the status endpoint to keep ourselves the active pilot.
    rospy.loginfo('Got access level: {}'.format(client.access_level))
    status_thread = start_pilot_update_thread(client)

    # Create a low-latency udp link for quickly sending messages to the vehicle.
    remote_address = client.get_udp_link_address()
    rospy.loginfo('Remote address: {}, udp port={}, id={}'.format(remote_address, udp_port,
                                                                  client.client_id))
    link = UDPLink(client.client_id, local_port=udp_port, remote_address=remote_address)

    # Subscribe to relevant messages, with the time delta as parameter
    link.subscribe('VEHICLE_POSE', 0.05)
    link.subscribe('GIMBAL_NAV_TRANSFORM_PB', 0.05)
    link.subscribe('VOXEL_OCCUPANCY_RUN_LENGTH_ENCODED_PB')

    # NOTE(teo): these are not piped through to ROS yet
    link.subscribe('EXTERNAL_WRENCH', 0.1)
    link.subscribe('WIND_SPEED', 0.1)
    link.subscribe('AIR_DENSITY', 1.0)

    #link.subscribe('VEHICLE_IMU', 0.05)

    # Connect the UDPLink to the vehicle before trying to takeoff.
    rospy.loginfo('Connecting...')
    link.connect()
    rospy.loginfo('Connected!')

    def handle_skydio_command(req):
        rospy.loginfo('Got command {}'.format(req.command))
        if req.command.lower() == 'prep':
            rospy.loginfo('Going to PREP')
            client.prep()
        elif req.command.lower() == 'takeoff':
            rospy.loginfo('Taking off')
            client.takeoff()
        elif req.command.lower() == 'set_skill':
            rospy.loginfo('Setting skill {}'.format(req.argument))
            client.set_skill(req.argument)
        elif req.command.lower() == 'start_mission':
            rospy.loginfo('Starting mission')

            skill_key = rospy.get_param('~inspection_skill', INSPECTION_SKILL)
            client.set_skill(skill_key)
            active_skill = client.get_active_skill()
            rospy.loginfo("Active skill: {}".format(active_skill))

            # Send start command to the skill
            req = dict(command='start')
            json_req = json.dumps(req)
            rospy.loginfo("Sending custom comms: {}".format(json_req))
            client.send_custom_comms(active_skill, json_req)

        elif req.command.lower() == 'land':
            rospy.loginfo('Landing')
            client.land()
        elif req.command.lower() == 'update_skillset':
            rospy.loginfo('Updating skillset')
            user_email = rospy.get_param('~user_email', None)
            if user_email:
                client.update_skillsets(user_email)
            else:
                rospy.logerr('Param user_email not set, cannot update skillset')

        return []

    svc = rospy.Service('~command', SkydioCommand, handle_skydio_command)

    # Create and run the proxy
    proxy = SkydioUdpProxy(udp_link=link,
                           http_client=client)

    # Start skill polling thread
    skill_thread = start_skill_thread(client, proxy)

    # Run the proxy loop
    proxy.run()


if __name__ == '__main__':
    main()

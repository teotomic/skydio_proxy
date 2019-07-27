from __future__ import absolute_import
from __future__ import print_function

import enum
import math
import json
import numpy as np

from shared.util.error_reporter import error_reporter as er
from shared.util.time_manager import time_manager as tm
from vehicle.skills.skills import Skill
from vehicle.skills.util import core
from vehicle.skills.util.core import AttrDict
from vehicle.skills.util.motions import CableMotion
from vehicle.skills.util.motions import GotoMotion
from vehicle.skills.util.motions import LookatMotion
from vehicle.skills.util.transform import Rot3
from vehicle.skills.util.transform import Transform

from .waypoints import WAYPOINTS_2D, ALTITUDES, OBSTACLE_MARGIN, WAYPOINTS

# Offset of the nav frame
#NAV_OFFSET = np.array([0, 0, 0.071])
NAV_OFFSET = np.array([0, 0, 0.0])

class InspectionState(enum.Enum):
    # Wait for the user to start the tour
    SETUP = 0

    # Execute the inspection
    INSPECT = 1

    # Return by backtracing
    #RETURN = 2

    # Paused
    PAUSED = 3

    # Stopped
    FINISHED = 5


class IndustrialInspection(Skill):
    """
    Fly to predefined points in the MRSM hall in Hessstr. 134
    """

    def __init__(self):
        super(IndustrialInspection, self).__init__()

        print("Initializing industrial inspection")
        self.command = None
        self.state = InspectionState.SETUP

        self.params = core.AttrDict()
        self.params.speed = 2.0
        self.params.min_turn_rate = 1  # [rad/update]
        self.params.max_turn_rate = 6  # [rad/update]
        self.params.distance_margin = 0.5  # [m]
        self.params.angle_margin = math.radians(0.75)  # [rad]
        self.params.giveup_utime = tm.seconds_to_utime(2.5)  # [s]
        self.params.giveup_speed = 0.1  # [m/s]

        # Waypoints
        self.motions = []
        self.waypoints_2d = WAYPOINTS_2D
        self.altitudes = ALTITUDES
        self.obstacle_margin = OBSTACLE_MARGIN
        self.waypoints = WAYPOINTS
        self.nav_offset = NAV_OFFSET
        self.motion_index = 0
        self.current_alternative_index = None

        # Fill in cable motions from the waypoints
        self.setup_motions()

    def setup_motions(self):
        print("Setting up motions")
        wp_trans = []
        for wp in self.waypoints:
            print("Adding point {} at altitude {}".\
                format(wp.position, wp.altitude if 'altitude' in wp else 'NONE'))

            pos_xy = self.waypoints_2d[wp.position].position
            if len(pos_xy) == 3:
                alt = pos_xy[2]
            else:
                alt = self.altitudes[wp.altitude]

            print("    Coords = {} {} {}, camera = {} {}".format(pos_xy[0], pos_xy[1], alt,
                  wp.camera.yaw, wp.camera.pitch))
            nav_T_wp = Transform(
                rotation=Rot3.Ypr(math.radians(wp.camera.yaw), math.radians(wp.camera.pitch), 0),
                translation=np.array([pos_xy[0], pos_xy[1], alt]) - self.nav_offset
            )
            wp_trans.append(self.trans_from_waypoint(wp))

        self.motions = []
        for curr_wp, next_wp in zip(wp_trans[:-1], wp_trans[1:]):
            self.motions.append(CableMotion(curr_wp, next_wp, self.params))

    def trans_from_waypoint(self, wp, alternative=None):
        if alternative is None or 'alternatives' not in wp:
            pos_xy = self.waypoints_2d[wp.position].position
        else:
            pos_xy = self.waypoints_2d[wp.alternatives[alternative]].position

        if len(pos_xy) == 3:
            alt = pos_xy[2]
        else:
            alt = self.altitudes[wp.altitude]

        print("    Coords = {} {} {}, camera = {} {}".format(pos_xy[0], pos_xy[1], alt,
              wp.camera.yaw, wp.camera.pitch))
        nav_T_wp = Transform(
            rotation=Rot3.Ypr(math.radians(wp.camera.yaw), math.radians(wp.camera.pitch), 0),
            translation=np.array([pos_xy[0], pos_xy[1], alt]) - self.nav_offset
        )
        return nav_T_wp

    def get_motion(self):
        if self.motion_index is None or self.motion_index >= len(self.motions):
            return None
        return self.motions[self.motion_index]

    def check_waypoint_alternatives(self, api):
        """
        Return True if it is ok to advance waypoint.
        """
        if self.motion_index >= len(self.waypoints) - 1:
            return True

        # Check altitude
        current_waypoint = self.waypoints[self.motion_index]
        next_waypoint = self.waypoints[self.motion_index + 1]

        # Get transforms from current and next waypoint
        wp_trans = self.trans_from_waypoint(current_waypoint, self.current_alternative_index)
        next_wp_trans = self.trans_from_waypoint(next_waypoint)

        if 'alternatives' in current_waypoint:
            pos = api.vehicle.get_position()

            # Check if we've reached the desired cable endpoint altitude
            if np.abs(pos[2] - next_wp_trans.translation()[2]) < 0.5:
                self.current_alternative_index = None
                return True;

            should_increment = isinstance(self.motions[self.motion_index], CableMotion)

            # Increment alternative
            if self.current_alternative_index is None:
                self.current_alternative_index = 0
            else:
                if should_increment:
                    self.current_alternative_index += 1
                if self.current_alternative_index >= len(current_waypoint.alternatives):
                    self.current_alternative_index = 0

            # Switch to alternative starting waypoint
            alt_wp = self.trans_from_waypoint(current_waypoint, self.current_alternative_index)
            if should_increment:
                self.motions[self.motion_index] = GotoMotion(alt_wp, self.params)
            else:
                self.motions[self.motion_index] = CableMotion(alt_wp, next_wp_trans, self.params)
            self.motions[self.motion_index].reset(api)

            return False

        self.current_alternative_index = None
        return True

    def update(self, api):
        # Don't allow subject tracking in this mode.
        api.subject.cancel_subject_tracking(api.utime)

        # Don't allow phone movements.
        api.phone.disable_movement_commands()

        # Set the upper-limit for vehicle speed
        api.movement.set_max_speed(4.0)

        # Set obstacle margin
        current_waypoint = self.waypoints[self.motion_index]
        if self.motion_index:
            if current_waypoint.obstacle_margin in self.obstacle_margin:
                margin = self.obstacle_margin[self.waypoints[self.motion_index].obstacle_margin]
            else:
                margin = 1.0
            api.planner.settings.obstacle_safety = margin
        else:
            api.planner.settings.obstacle_safety = 1.0

        if api.planner.is_landing():
            er.REPORT_STATUS("Exiting due to planner landing")
            api.skills.request_skill(api.utime, 'Basic')

        # Execute the current motion.
        if api.vehicle.get_pose():
            if self.state == InspectionState.INSPECT:
                motion = self.get_motion()
                if motion:
                    motion.update(api)
                    if motion.done:
                        if self.check_waypoint_alternatives(api):
                            motion.reset(api)
                            er.REPORT_STATUS("motion done, index ++ {}", self.motion_index)
                            self.motion_index += 1
                            er.REPORT_STATUS("flying to {}", self.waypoints[self.motion_index].name)
                        else:
                            er.REPORT_STATUS("Going to alternative {}",
                                             self.current_alternative_index)
                    else:
                        er.REPORT_STATUS("motion in progress")
                else:
                    # TODO: return if something went wrong - switch to RETURN
                    er.REPORT_STATUS("No more motions")
                    self.state = InspectionState.FINISHED
            else:
                er.REPORT_QUIET("Not active")
        else:
            er.REPORT_QUIET("Vehicle not ready")

    def handle_rpc(self, api, message):
        """ Process an incoming request and return the current execution state. """
        # Try to decode JSON
        try:
            data = AttrDict(json.loads(message))
            print("Got RPC request: {}".format(str(data)))
        except ValueError, e:
            print("Got RPC request but could not decode: {}".format(message))
            data = dict()

        if 'command' in data:
            if data.command.lower() == 'start':
                self.state = InspectionState.INSPECT
            elif data.command.lower() == 'pause':
                self.state = InspectionState.PAUSED
            elif data.command.lower() == 'setup':
                self.state = InspectionState.SETUP
                if 'waypoints' in data:
                    self.waypoints = data.waypoints

                if 'waypoints_2d' in data:
                    self.waypoints_2d = data.waypoints_2d

                if 'altitudes' in data:
                    self.altitudes = data.altitudes

                if 'obstacle_margin' in data:
                    self.obstacle_margin = data.obstacle_margin

                if 'nav_offset' in data:
                    self.nav_offset = data.nav_offset

                self.setup_motion()
                self.motion_index = 0

        response = dict(state=self.state.name,
                        motion_index=self.motion_index,
                        next_waypoint=WAYPOINTS[self.motion_index].name)
        return json.dumps(response)

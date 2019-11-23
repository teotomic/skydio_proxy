#!/usr/bin/python

import os
import json
import numpy as np

# This import registers the 3D projection, but is otherwise unused.
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import


class VoxelMap:
    def __init__(self):
        self.utime = -1
        self.origin = np.zeros(3)
        self.dims = np.zeros(3)
        self.scale = 0.0
        self.runs = []
        self.voxels = np.zeros(1)

    def _decode(self):
        self.voxels = np.zeros(self.dims[0] * self.dims[1] * self.dims[2])

        # Start with zeros
        value = 0
        start_inx = 0
        for run in self.runs:
            self.voxels[start_inx:start_inx + run] = value
            value = 0 if value else 1
            start_inx += run

        print(start_inx, self.voxels.shape)
        self.voxels = self.voxels.reshape(self.dims)
        print(self.voxels)

    @staticmethod
    def from_pb(msg, decode=False):
        voxel_map = VoxelMap()
        voxel_map.origin = np.array(msg.header.origin.data)
        voxel_map.dims = np.array(msg.header.dims.data)
        voxel_map.scale = np.array(msg.header.scale)
        voxel_map.runs = np.array(msg.runs)
        voxel_map.utime = msg.utime

        if decode:
            voxel_map._decode()

        return voxel_map

    @staticmethod
    def from_json(msg, decode=False):
        voxel_map = VoxelMap()
        voxel_map.origin = np.array(msg['data']['json']['header']['origin']['data'])
        voxel_map.dims = np.array(msg['data']['json']['header']['dims']['data'])
        voxel_map.scale = np.array(msg['data']['json']['header']['scale'])
        voxel_map.runs = np.array(msg['data']['json']['runs'])
        voxel_map.utime = msg['data']['json']['utime']

        if decode:
            voxel_map._decode()

        return voxel_map

    def __str__(self):
        return '<VoxelMap utime={} dims={} scale={} origin={}>'.format(self.utime, self.dims,
                                                                       self.scale, self.origin)

def show_voxel_map(voxel_map):
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.voxels(voxel_map.voxels, edgecolor='k')
    plt.show()

def test_voxel():
    with open('VOXEL_OCCUPANCY_RUN_LENGTH_ENCODED_T.json', 'r') as json_file:
        msg = json.load(json_file)
        voxel_map = VoxelMap.from_msg(msg)
        print(voxel_map)
        #show_voxel_map(voxel_map)

def test_pose():
    with open('VEHICLE_POSE.json', 'r') as json_file:
        msg = json.load(json_file)
        pose = VehiclePose.from_msg(msg)
        print(pose)

if __name__ == '__main__':
    test_voxel()
    test_pose()

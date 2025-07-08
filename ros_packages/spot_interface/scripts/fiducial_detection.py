#!/usr/bin/env python3

## @package spot_interface
# 
# \file fiducial_detection.py
# \brief It detects fiducials and publishes them on a ros topic
#
# author Zoe Betta
# version 1.0
# date 25/03/2024
# \details
# 
# Subscribes to: <BR>
#    None
#
# Publishes to: <BR>
#    /fiducials
#
# Description: <BR>
# this node receives from the robot information about the location
# of the detected fiducials, special markers, and publishes their location on a rostopic
# this node publishes the position in space and the ID of the marker
# 

from __future__ import print_function

from __future__ import absolute_import

import argparse
import collections
import logging
import sys
import time
import threading
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import rospy
import tf

import bosdyn
import bosdyn.client
import bosdyn.client.util

from bosdyn.api import world_object_pb2
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks

from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b,
                                         get_vision_tform_body, get_odom_tform_body)
from bosdyn.client.world_object import WorldObjectClient


from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from spot_interface.msg import fiducial

from tf import transformations
from spot_map.srv import Floor, FloorRequest

LOGGER = logging.getLogger(__name__)

def _update_thread(async_task):
    while True:
        async_task.update()
        time.sleep(0.01)

class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__("robot_state", robot_state_client, LOGGER,
                                              period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()


##
# \brief: function that implements the rosnode
# \param: 
# \return: None
#
# This functions is called when the node is initialized. It initializes the communication with
# the real robot by authenticating and connecting to the sdk. It then initialize an asynchronouas client 
# that reads the communication with the cameras and recognizes fiducials. The detected fiducials are then
# elaborated to be published on a rostopic with their position with respect to the odometry frame initialized
# at the startup of the robot. 
def main(argv):
    rospy.init_node('fiducialdetection')

    sdk = bosdyn.client.create_standard_sdk('FiducialClient')
    robot = sdk.create_robot('192.168.80.3')
    robot.authenticate('user', 'wruzvkg4rce4')
    #bosdyn.client.util.authenticate(robot)
    robot.sync_with_directory()

   # _point_cloud_client = robot.ensure_client('velodyne-point-cloud')
    _robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    _robot_state_task = AsyncRobotState(_robot_state_client)
    _world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
    _task_list = [_robot_state_task]
    _async_tasks = AsyncTasks(_task_list)

    update_thread = threading.Thread(target=_update_thread, args=[_async_tasks])
    update_thread.daemon = True
    update_thread.start()
    #_world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
    print('Connected.')
    # initialization of the publisher
    pub = rospy.Publisher('/fiducials', fiducial, queue_size=10)
    msg=fiducial()
    # initialization of tf
    listener = tf.TransformListener()
   
    #update_thread = threading.Thread(target=_update_thread, args=[_async_tasks])
    #update_thread.daemon = True
    #update_thread.start()

    # we initialize the rosservice to retrieve the floor the robot is on to make this compliant with the 
    # multi-floor exploration
    floor_service=rospy.ServiceProxy('retrievefloor', Floor)
    rospy.wait_for_service('retrievefloor')
    # Wait for the first responses.
    while not rospy.is_shutdown():
        # retrieve the list of detected fiducials
        request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        fiducial_objects = _world_object_client.list_world_objects(object_type=request_fiducials).world_objects
        # retrieve the position of the robot with respect to the odom frame
        try:
            (trans,rot) = listener.lookupTransform('/odom', BODY_FRAME_NAME, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        # retrieve the floor the robot is on
        req=FloorRequest()
        req.req=0
        fl=floor_service(req)
        floor=fl.floor
        # for each detected fiducial
        for i in fiducial_objects:
            # it sends the message with the name of the fiducial and its position
            name=i.apriltag_properties.frame_name_fiducial
            id_str=name.split("_")
            id_int=int(id_str[1])
            msg.id=id_int
            #retrieve position in frame odom
            vision_tform_fiducial = get_a_tform_b(i.transforms_snapshot, BODY_FRAME_NAME, i.apriltag_properties.frame_name_fiducial).to_proto()
            euler=transformations.euler_from_quaternion(rot)
            trMat=np.array([ [np.cos(euler[0])*np.cos(euler[1])*np.cos(euler[2])-np.sin(euler[0])*np.sin(euler[2]), -np.cos(euler[0])*np.cos(euler[1])*np.sin(euler[2])-np.sin(euler[0])*np.cos(euler[2]), np.cos(euler[0])*np.sin(euler[1]), trans[0]],
            [np.sin(euler[0])*np.cos(euler[1])*np.cos(euler[2])-np.cos(euler[0])*np.sin(euler[2]), -np.sin(euler[0])*np.cos(euler[1])*np.sin(euler[2])+np.cos(euler[0])*np.cos(euler[2]), np.sin(euler[0])*np.sin(euler[1]), trans[1]],
            [-np.sin(euler[1])*np.cos(euler[2]), np.sin(euler[1])*np.sin(euler[2]), np.cos(euler[1]), trans[2]],
            [0,0,0,1]
            ]
            )
            body_tform_fid=[vision_tform_fiducial.position.x, vision_tform_fiducial.position.y, vision_tform_fiducial.position.z, 1]
            start_tform_fiducial=np.matmul(trMat,body_tform_fid)
            # compile the message
            msg.x=start_tform_fiducial[0]
            msg.y=start_tform_fiducial[1]
            msg.z=start_tform_fiducial[2]
            msg.floor=floor
            print(msg.floor)
            pub.publish(msg)


if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)

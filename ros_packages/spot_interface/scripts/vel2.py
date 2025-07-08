#!/usr/bin/env python3

## @package spot_interface
# 
# \file vel2.py
# \brief this file reads the output of the velodyne sensors and publishes on a ros topic
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
#    /points2
#
#
# Description: <BR>
# This node registers to the Spot robot using credentials and initializes the client
# to read from the velodyne sensor. Data are received and elaborated
# in a ros compliant format to be subsequently published on the /points2 topic.





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

import bosdyn
import bosdyn.client
import bosdyn.client.util

from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks

from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.frame_helpers import get_odom_tform_body

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

matplotlib.use('Qt5agg')
LOGGER = logging.getLogger(__name__)
TEXT_SIZE = 10
SPOT_YELLOW = '#FBD403'

def _update_thread(async_task):
    while True:
        async_task.update()
        time.sleep(0.01)


class AsyncPointCloud(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client):
        super(AsyncPointCloud, self).__init__("point_clouds", robot_state_client, LOGGER,
                                              period_sec=0.2)

    def _start_query(self):
        return self._client.get_point_cloud_from_sources_async(["velodyne-point-cloud"])


class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__("robot_state", robot_state_client, LOGGER,
                                              period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()


""" def window_closed(ax):
    fig = ax.figure.canvas.manager
    active_managers = plt._pylab_helpers.Gcf.figs.values()
    return not fig in active_managers """


""" def set_axes_equal(ax):
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Args
      ax: a matplotlib axis, e.g., as output from plt.gca().
    

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])
 """

##
# \brief: function that implements the rosnode
# \param: 
# \return: None
#
# This functions is called when the node is initialized. It initializes the communication with
# the real robot by authenticating and connecting to the sdk. It then initialize an asynchronouas client 
# that reads the communication with the connected velodyne. The pointcloud received is then converted into 
# a ros compliant format, a PointCloud2 sensor_msg and published on the correct topic.  
def main(argv):
    #ros initialization
    rospy.init_node('pc2_publisher')
    pub = rospy.Publisher('points2', PointCloud2, queue_size=100)
    '''parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args(argv)'''
    # initialization of the robot communication
    sdk = bosdyn.client.create_standard_sdk('VelodyneClient')
    robot = sdk.create_robot('192.168.80.3')
    robot.authenticate('user', 'wruzvkg4rce4')
    #bosdyn.client.util.authenticate(robot)
    robot.sync_with_directory()
    # initialization of the pointcloud client for receiving data from the velodyne
    _point_cloud_client = robot.ensure_client('velodyne-point-cloud')
    _robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    _point_cloud_task = AsyncPointCloud(_point_cloud_client)
    _robot_state_task = AsyncRobotState(_robot_state_client)
    _task_list = [_point_cloud_task, _robot_state_task]
    _async_tasks = AsyncTasks(_task_list)
    print('Connected.')
    # creation of the thread
    update_thread = threading.Thread(target=_update_thread, args=[_async_tasks])
    update_thread.daemon = True
    update_thread.start()

    # Wait for the first responses.
    while any(task.proto is None for task in _task_list):
        time.sleep(0.1)
    fig = plt.figure()


    # Plot the point cloud as an animation.
    #ax = fig.add_subplot(111, projection='3d')
    aggregate_data = collections.deque(maxlen=1)
    while not rospy.is_shutdown():
        # if data are received compile the rostopic message to be sent
        if _point_cloud_task.proto[0].point_cloud:
            data = np.fromstring(_point_cloud_task.proto[0].point_cloud.data, dtype=np.float32)
            aggregate_data.append(data)
            plot_data = np.concatenate(aggregate_data)
            #print(plot_data[0::3])
            x=plot_data[0::3]
            y=plot_data[1::3]
            z=plot_data[2::3]
            # fill the intensity field with 1s since the velodyne does not send information about the intensity of the stream
            i=(x*0)+1
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('intensity', 12, PointField.FLOAT32, 1),]
            header = Header()
            header.frame_id = 'odom'
            header.stamp = rospy.Time.now()
            points = np.array([x,y,z,i]).reshape(4,-1).T
            pc2 = point_cloud2.create_cloud(header, fields, points)
            pub.publish(pc2)


if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)

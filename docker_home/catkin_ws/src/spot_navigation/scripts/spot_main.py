#!/usr/bin/env python3

## @package spot_navigation
#
# \file gotopoint.py
# \brief it implements the local navigation using the SpotSDK
#
# author Zoe Betta
# version 1.0
# date 27/03/2024
# \details
#
# Subscribes to: <BR>
#    goaltospot
#
# Publishes to: <BR>
#    None
#
# Description: <BR>
# This node receives the local navigation goal position, as a ros topic, and then
# sets the navigation parameter of the robot and sends the position that has to be reached.
#

import logging
import math
import signal
import sys
import threading
import time
from sys import platform
import numpy as np
import rospy

#!/usr/bin/env python3
from math import atan2, asin

import bosdyn.client
import bosdyn.client.util
from bosdyn import geometry
from bosdyn.api import geometry_pb2, image_pb2, trajectory_pb2, world_object_pb2
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from bosdyn.client.frame_helpers import (
    BODY_FRAME_NAME,
    VISION_FRAME_NAME,
    get_a_tform_b,
    get_vision_tform_body,
    ODOM_FRAME_NAME,
)
from bosdyn.client.lease import LeaseClient
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import (
    RobotCommandBuilder,
    RobotCommandClient,
    blocking_stand,
)
from bosdyn.client.robot_id import RobotIdClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.world_object import WorldObjectClient
from spot_msgs.msg import TrajectoryActionGoal
from geometry_msgs.msg import PoseStamped, Twist
from bosdyn.client.lease import Error as LeaseBaseError

from spot_navigation.srv import MoveLittle, MoveLittleResponse, MoveLittleRequest

LOGGER = logging.getLogger()
_robot_command_client = None
_robot = None
_powered_on = False
VELOCITY_BASE_ANGULAR = 0.8  # rad/sec
VELOCITY_CMD_DURATION = 2  # seconds
VELOCITY_BASE_SPEED = 0.5


##
# \brief: this function sets the navigation parameters
# \param: None
# \return: None
#
# This function sets the necessary parameters to complete a safe navigation
def set_mobility_params():
    obstacles = spot_command_pb2.ObstacleParams(
        disable_vision_body_obstacle_avoidance=False,
        disable_vision_foot_obstacle_avoidance=False,
        disable_vision_foot_constraint_avoidance=False,
        disable_vision_foot_obstacle_body_assist=False,
        disable_vision_negative_obstacles=False,
        obstacle_avoidance_padding=0.2,
    )

    footprint_R_body = geometry.EulerZXY()
    position = geometry_pb2.Vec3(x=0.0, y=0.0, z=0.0)
    rotation = footprint_R_body.to_quaternion()
    pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
    point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
    traj = trajectory_pb2.SE3Trajectory(points=[point])
    body_control = spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)

    speed_limit = SE2VelocityLimit(
        max_vel=SE2Velocity(linear=Vec2(x=1.0, y=1.0), angular=0.7)
    )

    mobility_params = spot_command_pb2.MobilityParams(
        obstacle_params=obstacles,
        vel_limit=speed_limit,
        body_control=body_control,
        locomotion_hint=spot_command_pb2.HINT_AUTO,
    )
    return mobility_params


##
# \brief: this function sets the navigation parameters
# \param x: first term of the quaternion representation
# \param y: second term of the quaternion representation
# \param z: third term of the quaternion representation
# \param w: fourth term of the quaternion representation
#
# \return roll_x: roll angle in radians
# \return pitch_y: pitch angle in radians
# \return yaw_z: yaw angle in radians
#
# This function sgiven a quaternion representation of a rotation transforms
# it into the roll-pitch-yaw representation
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


def goal_clbk(goal):
    global mobility_params, _robot_command_client
    orientation = euler_from_quaternion(
        goal.pose.orientation.x,
        goal.pose.orientation.y,
        goal.pose.orientation.z,
        goal.pose.orientation.w,
    )
    mobility_params = set_mobility_params()
    tag_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
        goal_x=goal.pose.position.x,
        goal_y=goal.pose.position.y,
        goal_heading=orientation[2],
        frame_name=ODOM_FRAME_NAME,
        params=mobility_params,
        body_height=0.0,
        locomotion_hint=spot_command_pb2.HINT_AUTO,
    )
    end_time = 10.0
    rospy.loginfo("command ready")
    _robot_command_client.robot_command(
        lease=None, command=tag_cmd, end_time_secs=time.time() + end_time
    )


def power_on():
    global _robot, _powered_on
    _robot.power_on()
    _powered_on = True


def power_off():
    safe_power_off_cmd = RobotCommandBuilder.safe_power_off_command()
    _robot_command_client.robot_command(command=safe_power_off_cmd)
    time.sleep(2.5)
    _powered_on = False


def move_service(req):
    print(req)
    if req.req == 1:
        turn_left()
    if req.req == 2:
        turn_right()
    if req.req == 3:
        go_back()
    if req.req == 4:
        move_right()
    if req.req == 5:
        move_left()
    if req.req == 6:
        go_forward()
    return MoveLittleResponse(True)


def move_right():
    _velocity_cmd_helper("go_back", v_y=VELOCITY_BASE_SPEED, dur=1)


def move_left():
    _velocity_cmd_helper("go_back", v_y=-VELOCITY_BASE_SPEED, dur=1)


def go_forward():
    _velocity_cmd_helper("go_back", v_x=VELOCITY_BASE_SPEED, dur=1)


def go_back():
    _velocity_cmd_helper("go_back", v_x=-VELOCITY_BASE_SPEED, dur=1)


def turn_right():
    _velocity_cmd_helper("turn_right", v_rot=VELOCITY_BASE_ANGULAR)


def turn_left():
    _velocity_cmd_helper("turn_left", v_rot=-VELOCITY_BASE_ANGULAR)


def _velocity_cmd_helper(desc="", v_x=0.0, v_y=0.0, v_rot=0.0, dur=2):
    _start_robot_command(
        desc,
        RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot),
        end_time_secs=time.time() + VELOCITY_CMD_DURATION,
    )


def _start_robot_command(desc, command_proto, end_time_secs=None):
    def _start_command():
        global mobility_params, _robot_command_client
        _robot_command_client.robot_command(
            command=command_proto, end_time_secs=end_time_secs
        )

    _try_grpc(desc, _start_command)


def _try_grpc(desc, thunk):
    try:
        return thunk()
    except (ResponseError, RpcError, LeaseBaseError) as err:
        print(f"Failed {desc}: {err}")
        return None

import os
DEFAULT_SPOT_IP_ADDRESS = "192.168.80.3"
DEFAULT_SPOT_USERNAME = "user"
DEFAULT_SPOT_PASSWORD = "password"
spot_username = os.getenv("SPOT_USERNAME",DEFAULT_SPOT_USERNAME)
spot_password = os.getenv("SPOT_PASSWORD",DEFAULT_SPOT_PASSWORD)
spot_ip_address = os.getenv("SPOT_IP_ADDRESS",DEFAULT_SPOT_IP_ADDRESS)
def node():
    global goal, mobility_params, _robot, _robot_id, _power_client, _robot_state_client, _robot_command_client
    rospy.init_node("timeretrieve", anonymous=False)

    sdk = create_standard_sdk("gotopoint")
    robot = sdk.create_robot(spot_ip_address)
    print(f'''spot_ip_address = {spot_ip_address}''')
    print(f'''spot_username = {spot_username}''')
    robot.authenticate(spot_username, spot_password)

    rospy.Subscriber("goaltospot", PoseStamped, goal_clbk)
    robot.time_sync.wait_for_sync()

    _robot = robot
    _robot_id = robot.ensure_client(RobotIdClient.default_service_name).get_id(
        timeout=0.4
    )
    _lease_client = robot.ensure_client(LeaseClient.default_service_name)
    _power_client = robot.ensure_client(PowerClient.default_service_name)
    _robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    _robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)

    mobility_params = set_mobility_params()
    _lease = _lease_client.take()
    _lease_keepalive = bosdyn.client.lease.LeaseKeepAlive(
        _lease_client, must_acquire=True, return_at_exit=True
    )

    s = rospy.Service("move_little", MoveLittle, move_service)

    power_on()
    time.sleep(5)
    blocking_stand(_robot_command_client)
    time.sleep(5)
    # rospy.loginfo("ready to take commands")
    # turn_left()
    # time.sleep(2)
    # turn_left()
    # time.sleep(2)
    # turn_left()
    # time.sleep(2)
    # turn_left()
    # rospy.loginfo("maybe turned a bit left?")
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        rospy.spin()
    power_off()


if __name__ == "__main__":
    try:
        node()
    except rospy.ROSInterruptException:
        pass

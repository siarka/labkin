#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2021 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import time
import threading
import socket

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

# Parametry komunikacji UDP
udp_host = "localhost"  # Adres hosta, z którego odbieramy dane
udp_port = 12345  # Port, na którym odbieramy dane
buffer_size = 1024  # Rozmiar bufora odbiorczego
# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 30

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    def check(notification, e=e):
        print("EVENT: " + Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()

    return check

def example_move_to_home_position(base):
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle is None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e), Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

def populateCartesianCoordinate(waypointInformation):
    waypoint = Base_pb2.CartesianWaypoint()
    waypoint.pose.x = waypointInformation[0]
    waypoint.pose.y = waypointInformation[1]
    waypoint.pose.z = waypointInformation[2]
    waypoint.blending_radius = waypointInformation[3]
    waypoint.pose.theta_x = waypointInformation[4]
    waypoint.pose.theta_y = waypointInformation[5]
    waypoint.pose.theta_z = waypointInformation[6]
    waypoint.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE

    return waypoint
def example_trajectory_from_file(base, base_cyclic):

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((udp_host, udp_port))

    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    product = base.GetProductConfiguration()

    waypoints = Base_pb2.WaypointList()
    waypoints.duration = 0.0
    waypoints.use_optimal_blending = False
    while True:
        try:
            data, addr = udp_socket.recvfrom(buffer_size)
            data = data.decode()  # Dekoduj dane z bajtów do tekstu

            waypoint = waypoints.waypoints.add()
            waypoint.name = "waypoint_0"
            waypoint.cartesian_waypoint.CopyFrom(populateCartesianCoordinate(data))

            result = base.ValidateWaypointList(waypoints)
            if len(result.trajectory_error_report.trajectory_error_elements) == 0:
                e = threading.Event()
                notification_handle = base.OnNotificationActionTopic(
                    check_for_end_or_abort(e), Base_pb2.NotificationOptions()
                )

                print("Moving cartesian trajectory...")

                base.ExecuteWaypointTrajectory(waypoints)

                print("Waiting for trajectory to finish ...")
                finished = e.wait(TIMEOUT_DURATION)
                base.Unsubscribe(notification_handle)

                if finished:
                    print("Cartesian trajectory with no optimization completed")
                    e_opt = threading.Event()
                    notification_handle_opt = base.OnNotificationActionTopic(
                        check_for_end_or_abort(e_opt), Base_pb2.NotificationOptions()
                    )

                    waypoints.use_optimal_blending = True
                    base.ExecuteWaypointTrajectory(waypoints)

                    print("Waiting for trajectory to finish ...")
                    finished_opt = e_opt.wait(TIMEOUT_DURATION)
                    base.Unsubscribe(notification_handle_opt)

                    if finished_opt:
                        print("Cartesian trajectory with optimization completed")
                    else:
                        print("Timeout on action notification wait for optimized trajectory")

                    return finished_opt
                else:
                    print("Timeout on action notification wait for non-optimized trajectory")
            else:
                print("Error found in trajectory")
                result.trajectory_error_report.PrintDebugString()
        except KeyboardInterrupt:
             # Obsługa przerwania przez użytkownika (Ctrl+C)
            print("Received Ctrl+C. Exiting.")
            udp_socket.close()
            break


def main():
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    args = utilities.parseConnectionArguments()

    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        success = True
        success &= example_move_to_home_position(base)
        success &= example_trajectory_from_file(base, base_cyclic)

        return 0 if success else 1

if __name__ == "__main__":
    exit(main())

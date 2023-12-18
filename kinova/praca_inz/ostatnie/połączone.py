#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import threading
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Session_pb2, Base_pb2

import time
import argparse
import cv2 as cv
from pupil_apriltags import Detector
import numpy as np
import socket
import math

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--families", type=str, default='tag36h11')
    parser.add_argument("--nthreads", type=int, default=1)
    parser.add_argument("--quad_decimate", type=float, default=2.0)
    parser.add_argument("--quad_sigma", type=float, default=0.0)
    parser.add_argument("--refine_edges", type=int, default=1)
    parser.add_argument("--decode_sharpening", type=float, default=0.25)
    parser.add_argument("--debug", type=int, default=0)

    args = parser.parse_args()

    return args

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20


# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """

    def check(notification, e=e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
                or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()

    return check

def oblicz_odleglosc(punkt1, punkt2):
    x1, y1 = punkt1
    x2, y2 = punkt2
    odleglosc = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return odleglosc

def prostok(a):
    return math.sqrt(a*a+0.01)


def det_tags(
    image,
    tags,
    dt,
    pre_err,
    end_points,
    base
):
    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    c_u = 311.753418
    c_v = 232.400955
    f_x = 653.682312
    f_y = 651.856018
    a = 4/3
    b = [0, 0, 0, 0]
    stala_do_dl_x = 160*f_x
    stala_do_dl_y = 160*f_y
    z = np.array([1, 1, 1, 1])
    wykryte = False
    poch_err = np.array([0, 0, 0, 0, 0, 0, 0, 0])

    mn = np.array([1 / 1000, 1 / 1000, 1 / 1000, 180.0 / math.pi, 180.0 / math.pi, 180.0 / math.pi])
    #mnożnik parametrów do sterowanie 1-3 m/s 4-6 deg/s

    for tag in tags:
        corners = tag.corners
        wykryte = True
        corner_01 = (int(corners[0][0]), int(corners[0][1]))
        corner_02 = (int(corners[1][0]), int(corners[1][1]))
        corner_03 = (int(corners[2][0]), int(corners[2][1]))
        corner_04 = (int(corners[3][0]), int(corners[3][1]))

    if wykryte:
        print("zostały wykryte tagi")
        b[0] = stala_do_dl_x / oblicz_odleglosc(corner_01, corner_02)
        b[1] = stala_do_dl_y / oblicz_odleglosc(corner_03, corner_02)
        b[2] = stala_do_dl_x / oblicz_odleglosc(corner_03, corner_04)
        b[3] = stala_do_dl_y / oblicz_odleglosc(corner_01, corner_04)

        z[0] = (prostok(b[0]) + prostok(b[3])) / 2
        z[1] = (prostok(b[0]) + prostok(b[1])) / 2
        z[2] = (prostok(b[2]) + prostok(b[1])) / 2
        z[3] = (prostok(b[2]) + prostok(b[3])) / 2

        cur_points = np.array([
            (corner_01[0] - c_u) / (f_x * a), (corner_01[1] - c_v) / f_y,
            (corner_02[0] - c_u) / (f_x * a), (corner_02[1] - c_v) / f_y,
            (corner_03[0] - c_u) / (f_x * a), (corner_03[1] - c_v) / f_y,
            (corner_04[0] - c_u) / (f_x * a), (corner_04[1] - c_v) / f_y
        ])

        err = cur_points - end_points
        #if (abs(err[0])+abs(err[1])+abs(err[2])+abs(err[3])+abs(err[4])+abs(err[5])+abs(err[6])+abs(err[7])) < 1:
         #   err = np.array([0, 0, 0, 0, 0, 0, 0, 0])

        poch_err = (pre_err - err)/dt

        L = np.array([
            [-1/z[0], 0, cur_points[0]/z[0], cur_points[0]*cur_points[1], -1-(cur_points[0]*cur_points[0]), cur_points[1]],
            [0, -1/z[0], cur_points[1]/z[0], 1+(cur_points[1]*cur_points[1]), -cur_points[0]*cur_points[1], -cur_points[0]],
            [-1/z[1], 0, cur_points[2]/z[1], cur_points[2]*cur_points[3], -1-(cur_points[2]*cur_points[2]), cur_points[3]],
            [0, -1/z[1], cur_points[3]/z[1], 1+(cur_points[3]*cur_points[3]), -cur_points[2]*cur_points[3], -cur_points[2]],
            [-1/z[2], 0, cur_points[4]/z[2], cur_points[4]*cur_points[5], -1-(cur_points[4]*cur_points[4]), cur_points[5]],
            [0, -1/z[2], cur_points[5]/z[2], 1+(cur_points[5]*cur_points[5]), -cur_points[4]*cur_points[5], -cur_points[4]],
            [-1/z[3], 0, cur_points[6]/z[3], cur_points[6]*cur_points[7], -1-(cur_points[6]*cur_points[6]), cur_points[7]],
            [0, -1/z[3], cur_points[7]/z[3], 1+(cur_points[7]*cur_points[7]), -cur_points[6]*cur_points[7], -cur_points[6]]
        ])

        pi_L = np.linalg.pinv(L)

        result = np.dot(pi_L, poch_err)
        result = np.round(result, 4)
        result = result * mn
    else:
        err = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        print(" wymuszone 0 ")
        result = np.array([0, 0, 0, 0, 0, 0])

    ster = f"{result[0]} {result[1]} {result[2]} {result[3]} {result[4]} {result[5]}"
    # Wysyłanie do hosta i portu za pomocą protokołu UDP
    print("sterowanie to :")
    print(ster)
    try:
        if len(ster) == 6:
            twist = command.twist
            twist.linear_x = ster[0]
            twist.linear_y = ster[1]
            twist.linear_z = ster[2]
            twist.angular_x = ster[3]
            twist.angular_y = ster[4]
            twist.angular_z = ster[5]
            base.SendTwistCommand(command)
        else:
            print("Nieprawidłowy format danych: ", ster)

    except KeyboardInterrupt:
        print("Received Ctrl+C. Exiting.")
        base.Stop()
    except Exception as e:
        print(f"Error: {e}")

    return err


def main():
    args = get_args()

    families = args.families
    nthreads = args.nthreads
    quad_decimate = args.quad_decimate
    quad_sigma = args.quad_sigma
    refine_edges = args.refine_edges
    decode_sharpening = args.decode_sharpening
    debug = args.debug

    cap = cv.VideoCapture("../recording-20231102095508.ts")
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))
    import utilities

    # Parse arguments
    args_rob = utilities.parseConnectionArguments()

    at_detector = Detector(
        families=families,
        nthreads=nthreads,
        quad_decimate=quad_decimate,
        quad_sigma=quad_sigma,
        refine_edges=refine_edges,
        decode_sharpening=decode_sharpening,
        debug=debug,
    )

    dt = 1
    f_x = 653.682312
    f_y = 651.856018
    a = 4/3
    c_u = 311.753418
    c_v = 232.400955

    end_points = np.array([
        (469-c_u)/(f_x*a), (112-c_v)/f_y,
        (116-c_u)/(f_x*a), (107-c_v)/f_y,
        (127-c_u)/(f_x*a), (443-c_v)/f_y,
        (456-c_u)/(f_x*a), (445-c_v)/f_y
    ])
    pre_err = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    while True:
        start_time = time.time()

        ret, image = cap.read()
        if not ret:
            break

        image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        tags = at_detector.detect(
            image,
            estimate_tag_pose=False,
            camera_params=None,
            tag_size=None,
        )
        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(args_rob) as router:
            # Create required services
            base = BaseClient(router)

            # Example core
            success = True
            pre_err /= 2
            pre_err += det_tags(image, tags, dt, pre_err, end_points,base)
            end_time = time.time()
            dt = end_time - start_time
            key = cv.waitKey(1)
            if key == 27:  # ESC
                break

    cap.release()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()

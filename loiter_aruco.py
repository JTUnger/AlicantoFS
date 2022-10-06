import time
import socket
import math
import argparse
import threading
import sys

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
from pymavlink import mavutil

import cv2
import cv2.aruco as aruco
import numpy as np

from imutils.video import WebcamVideoStream
import imutils

def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect
    baud_rate = 57600

    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    return vehicle

def safety_exit(vehicle):
    """ Run in parallel thread to check for RC controler interupt."""
    if vehicle.mode != "GUIDED" or vehicle.mode != "LAND":
        print("Non-standard flight mode detected. Exiting script.")
        sys.exit()



def loiter_aruco(vehicle, loiterAltitud, timeout_seconds = 0, marker_id = 72, marker_size = 19,search_time_seconds = 60):
    """Loiter over selected aruco marker until timeout at loiter altitude."""
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()

    horizontal_res = 800
    vertical_res = 600

    horizontal_fov =59  * (math.pi / 180 ) 
    vertical_fov =46  * (math.pi / 180)

    cameraMatrix   = np.array([[933.9466925521707, 0, 358.59608943398365],
                [0, 935.0635463990791, 293.0721064675127],
                [0, 0, 1]])
    cameraDistortion   = np.array([-0.4530790972005633, 0.3951099938612813, 
                                0.0037673873203789916, 0.0016363264710513889,
                                -0.38177331299300393])
    cap = WebcamVideoStream(src=0, width=horizontal_res, height=vertical_res).start()

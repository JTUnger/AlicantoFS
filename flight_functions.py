
from re import search
from dronekit import *
import time
import socket
import math
import argparse
from exceptions import *
from pymavlink import mavutil
import cv2
import cv2.aruco as aruco
import numpy as np
from imutils.video import WebcamVideoStream
import imutils

def connectMyCopter():
    """ Connects to drone and returns vehicle object, takes argument "connect" 
    to specify connection parameter, default is RPI3 to PIXHAWK (/dev/ttyAMA0) """
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string = args.connect
    baud_rate = 57600

    if not connection_string:
        connection_string = "/dev/ttyAMA0"

    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    return vehicle

def arm_takeoff(vehicle, targetAltitude):
    """Waits for drone to be armable, arms motors and takes off to target altitude"""
    while vehicle.is_armable != True:
        print("Waiting for vehicle to become armable")
        time.sleep(1)
    
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    
    vehicle.armed = True
    while vehicle.armed == False:
        print("Waiting for arming...")
        time.sleep(1)

    vehicle.simple_takeoff(targetAltitude)
    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= targetAltitude*0.95:
            print("Target altitude reached")
            break
        time.sleep(1)

def get_distance_meters(targetloc, currentloc):
    """Returns distance in meters between two LocationGlobalRelative objects,
     planar aproximation, not acurate for long distances"""
    dLat= targetloc.lat - currentloc.lat
    dLon= targetloc.lon - currentloc.lon
    return math.sqrt((dLon*dLon) + (dLat*dLat))*1.113195e5

def goto(vehicle, targetloc, safety_distance = 500):
    """Moves drone to target location. Takes vehicle object, target location and safety distance as parameters.
    Safety distance is the maximum distance the vehicle is allowed to move using the goto function, if the distance 
    is greater drone will not execute command and SecurityError will be raised. Default safety distance is 500m"""

    targetdist= get_distance_meters(targetloc, vehicle.location.global_relative_frame)
    if targetdist > safety_distance:
        raise SecurityError("Target distance is greater than safety distance")

    vehicle.simple_goto(targetloc)
    while vehicle.mode.name=='GUIDED':
        currentdist=  get_distance_meters(targetloc, vehicle.location.global_relative_frame)
        if currentdist < targetdist*0.01:
            print('target reached')
            time.sleep(2)
            break
        time.sleep(1)
    
def set_altitude(vehicle, targetAltitude):
    """"Sets vehicle height, does not move vehicle."""
    #gets current location
    currentloc = vehicle.location.global_relative_frame
    vehicle.simple_goto(LocationGlobalRelative(currentloc.lat, currentloc.lon, targetAltitude))   
    #checks if altitude is reached
    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= targetAltitude*0.95:
            print("Target altitude reached")
            break
        time.sleep(1)   

def send_local_ned_velocity(vehicle, vx, vy, vz):
    """ Sends velocity commands to drone, takes vehicle object, vx, vy, vz as parameters. 
    Velocity commands are in NED frame relative to drone. Velocity commands in m/s"""
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,0,0,
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111,
		0,0,0,
		vx,vy,vz,
		0,0,0,
		0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()	

def send_global_ned_velocity(vehicle, vx, vy, vz):
    """ Sends velocity commands to drone, takes vehicle object, vx, vy, vz as parameters. 
    Velocity commands are in NED frame relative to global NED. Velocity commands in m/s"""
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,0,0,
		mavutil.mavlink.MAV_FRAME_LOCAL_NED,
		0b0000111111000111,
		0,0,0,
		vx,vy,vz,
		0,0,0,
		0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()	

def send_land_message(vehicle, x, y):
    """ Sends encoded landing target position to drone, if vehicle is in LAND mode 
    and recives landing target position, precision landing will be enabled."""
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,
        0,
        0,)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def aruco_precision_landing(vehicle, id_aruco = 72, size_aruco_cm = 19, 
                            max_landing_attempts = 2, start_height = 8, safety_height = 15):
    """ Starts precision landing logic on aruco marker. Takes vehicle object, id of aruco marker, 
    size of aruco marker and number of landing attempts as parameters. Call this function ONCE, NOT in a loop.
    Vehicle will come down to start height and then start precision landing. If precision landing fails, drone will go to 
    safety altitude and a LandingError exception will be raised."""

    #Aruco Dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()

    #Camera Parameters
    horizontal_res = 800
    vertical_res = 600
    horizontal_fov = 59 * (math.pi/180)
    vertical_fov = 46 * (math.pi/180)

    #Start Video Capture
    capture = WebcamVideoStream(width = horizontal_res, height = vertical_res).start()

    #Camera Intrinsics
    cameraMatrix   = np.array([[933.9466925521707, 0, 358.59608943398365],
                [0, 935.0635463990791, 293.0721064675127],
                [0, 0, 1]])
    cameraDistortion   = np.array([-0.4530790972005633, 0.3951099938612813, 0.0037673873203789916, 
                                    0.0016363264710513889, -0.38177331299300393])

    #Set Vehicle Parameters

    vehicle.parameters['PLND_ENABLED'] = 1
    vehicle.parameters['PLND_TYPE'] = 1 
    vehicle.parameters['PLND_EST_TYPE'] = 0 ##0 for raw sensor, 1 for kalman filter pos estimation
    vehicle.parameters['LAND_SPEED'] = 20 ##Descent speed of 20cm/s

    #Precision Landing Logic

    landing_attempts = 0

    while landing_attempts < max_landing_attempts:


        print("Landing attempt: ", landing_attempts + 1)

        vehicle.mode = VehicleMode("GUIDED")
        while vehicle.mode.name != 'GUIDED':
            time.sleep(0.5)
        set_altitude(vehicle, start_height)

        found_count = 0
        notfound_count = 0
        search_frames = 0 
        land = False

        #Vehicle is hovering at start_height and looks for aruco marker for 15 frames, 
        # if found in one frame precision landing is started, else atempt in marked as failed.

        while search_frames < 15:
            #Capture and flip image
            frame = capture.read()
            frame = cv2.flip(frame, 0)
            frame = cv2.flip(frame, 1)
            frame = cv2.resize(frame, (horizontal_res, vertical_res))
            frame_np = np.array(frame)
            gray_img = cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)
            ids=''
            corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)

            #If aruco is found add to found_count
            if ids is not None and ids[0] == id_aruco:
                found_count += 1

            #Aruco is found in one frame, precision landing is started
            if found_count > 0:
                land = True
                break 

            #If aruco is not found in the 15 frame search window, atempt is marked as failed
            if search_frames == 14 and found_count == 0:
                print('Aruco not found, precision landing failed')
                landing_attempts += 1
                land = False
                break
            search_frames += 1
        
        #Precision Landing begins if aruco is found in one frame
        if land == True:
            print('Aruco found, precision landing started')
            while vehicle.armed == True:
                abort_counter = 0
                #Capture frame and flip to correct orientation and prep
                frame = capture.read()
                frame = cv2.flip(frame, 0)
                frame = cv2.flip(frame, 1)
                frame = cv2.resize(frame, (horizontal_res, vertical_res))
                frame_np = np.array(frame)
                gray_img = cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)
                ids=''
                corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)

                #Set vehicle mode to LAND if vehicle is in GUIDED mode
                if vehicle.mode.name == 'GUIDED':
                    vehicle.mode = VehicleMode("LAND")
                    while vehicle.mode.name != 'LAND':
                        time.sleep(0.5)
                
                try:
                    #Aruco is found
                    if ids is not None and ids[0] == id_aruco: 
                        ret = aruco.estimatePoseSingleMarkers(corners,size_aruco_cm,cameraMatrix=cameraMatrix,distCoeffs=cameraDistortion)
                        (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
                        x = '{:.2f}'.format(tvec[0])
                        y = '{:.2f}'.format(tvec[1])
                        z = '{:.2f}'.format(tvec[2])

                        x_sum = 0
                        y_sum = 0

                        x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
                        y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]

                        x_center = x_sum/4
                        y_center = y_sum/4

                        x_angle = (x_center - horizontal_res/2) * (horizontal_fov / horizontal_res)
                        y_angle = (y_center - vertical_res/2) * (vertical_fov / vertical_res)

                        send_land_message(vehicle, x_angle, y_angle)

                        found_count += 1
                        abort_counter = 0

                        print("X CENTER OF ARUCO: ", x_center + "Y CENTER OF ARUCO: ", y_center)
                        print("FOUND COUNT: ", found_count + "NOT FOUND COUNT: ", notfound_count)
                        print("MARKER X POSITION: ", x + "MARKER Y POSITION: ", y + "MARKER Z POSITION: ", z)
                        print("\n")

                    #Aruco is not found
                    else:
                        notfound_count += 1
                        abort_counter += 1
                        #If aruco is not found for 15 consecutive frames(~1.5 second), precision landing is aborted
                        if abort_counter > 15:
                            print("Target lost, precision landing aborted")
                            vehicle.mode = VehicleMode("GUIDED")
                            while vehicle.mode != "GUIDED":
                                time.sleep(0.5)
                            landing_attempts += 1
                            break  

                except Exception as e:
                    print("Target probably not found. Error: ", e)
                    notfound_count += 1
                    abort_counter += 1

    #If all landing attempts are completed, and vehicle in still armed, 
    # raise LandingError exception and go to safe altitude and set mode to guided
    if vehicle.armed == True:
        set_altitude(vehicle, safety_height)
        vehicle.mode = VehicleMode("GUIDED")
        while vehicle.mode != "GUIDED":
            time.sleep(0.5)
        print("All landing attempts completed, vehicle still armed, aborting")
        raise LandingError("Precision Landing Failed")

def loiter_aruco(vehicle, loiter_height, loiter_time, safety_height = 15, id_aruco = 72, size_aruco_cm = 20):
    """Vehicle comes down to loiter_height and looks for aruco marker with the correct id_aruco, if marker is found,
    vehicle moves to marker and loiters at loiter_height. Vehicle loiters for loiter_time seconds and then sets its mode to guided.
    If aruco marker is not found, vehicle goes to safety_height, sets mode to guided and raises LoiterError exception."""

    #Aruco Dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()

    #Camera Parameters
    horizontal_res = 800
    vertical_res = 600
    horizontal_fov = 59 * (math.pi/180)
    vertical_fov = 46 * (math.pi/180)

    #Start Video Capture
    capture = WebcamVideoStream(width = horizontal_res, height = vertical_res).start()

    #Camera Intrinsics
    cameraMatrix   = np.array([[933.9466925521707, 0, 358.59608943398365],
                [0, 935.0635463990791, 293.0721064675127],
                [0, 0, 1]])
    cameraDistortion   = np.array([-0.4530790972005633, 0.3951099938612813, 0.0037673873203789916, 
                                    0.0016363264710513889, -0.38177331299300393])

    #Set Vehicle Parameters

    vehicle.parameters['PLND_ENABLED'] = 1  #Enable Precision Landing
    vehicle.parameters['PLND_TYPE'] = 1     #1 for companion computer
    vehicle.parameters['PLND_EST_TYPE'] = 0 #0 for raw sensor, 1 for kalman filter pos estimation

    #Precision Loiter Logic

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        time.sleep(0.5)
    set_altitude(vehicle, loiter_height)

    found_count = 0
    notfound_count = 0
    search_frames = 0
    loiter = False

    ##Vehicle is hovering at start_height and looks for aruco marker for 15 frames, 
    # if found in one frame precision loiter is started, else atempt in marked as failed.  
    
    while search_frames < 15:
        #Capture frame and flip to correct orientation and prep
        frame = capture.read()
        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (horizontal_res, vertical_res))
        frame_np = np.array(frame)
        gray_img = cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)
        ids=''
        corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)
        
        #If aruco is found add to found_count
        if ids is not None and ids[0] == id_aruco:
            found_count += 1

        #Aruco is found in one frame, precision landing is started
        if found_count > 0:
            loiter = True
            break 

        #If aruco is not found in the 15 frame search window, atempt is marked as failed
        if search_frames == 14 and found_count == 0:
            print('Aruco not found, precision loiter failed')
            loiter = False
            break
        search_frames += 1

    #If aruco is found, vehicle moves to marker and loiters at loiter_height for loiter_time seconds
    if loiter == True:
        print("Aruco found, precision loiter started")
        vehicle.mode = VehicleMode("LOITER")
        while vehicle.mode != "LOITER":
            time.sleep(0.5)

        timeout = time.time() + loiter_time
        while time.time() < timeout:
            #Capture frame and flip to correct orientation and prep
            frame = capture.read()
            frame = cv2.flip(frame, 0)
            frame = cv2.flip(frame, 1)
            frame = cv2.resize(frame, (horizontal_res, vertical_res))
            frame_np = np.array(frame)
            gray_img = cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)
            ids=''
            corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)

            try: 
                #Aruco is found
                if ids is not None and ids[0] == id_aruco:
                    ret = aruco.estimatePoseSingleMarkers(corners, size_aruco_cm, cameraMatrix, cameraDistortion)
                    (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
                    x = '{:.2f}'.format(tvec[0])
                    y = '{:.2f}'.format(tvec[1])
                    z = '{:.2f}'.format(tvec[2])

                    x_sum = 0
                    y_sum = 0

                    x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
                    y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]

                    x_center = x_sum/4
                    y_center = y_sum/4

                    x_angle = (x_center - horizontal_res/2) * (horizontal_fov / horizontal_res)
                    y_angle = (y_center - vertical_res/2) * (vertical_fov / vertical_res)

                    send_land_message(vehicle, x_angle, y_angle)

                    found_count += 1
                    abort_counter = 0

                    print("X CENTER OF ARUCO MARKER: ", x_center + " Y CENTER OF ARUCO MARKER: ", y_center)
                    print("FOUND COUNT: ", found_count + " NOT FOUND COUNT: ", notfound_count)
                    print("\n")

                #Aruco is not found
                else:
                    notfound_count += 1
                    abort_counter += 1
                    if abort_counter > 15:
                        print("Aruco marker lost, precision loiter failed")
                        vehicle.mode = VehicleMode("GUIDED")
                        while vehicle.mode != "GUIDED":
                            time.sleep(0.5)
                        set_altitude(vehicle, safety_height)
                        loiter = False
                        break
            except Exception as e:
                print(e)
                pass

    #If aruco is not found, vehicle goes to safety_height, sets mode to guided and raises LoiterError exception
    if loiter == False:
        vehicle.mode = VehicleMode("GUIDED")
        while vehicle.mode != "GUIDED":
            time.sleep(0.5)
        set_altitude(vehicle, safety_height)
        print("Aruco probably not found, precision loiter failed")
        raise LoiterError("Precision Loiter Failed")
        
    
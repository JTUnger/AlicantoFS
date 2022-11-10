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
import datetime
import os

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

def goto_local_frame(vehicle, north, east, down):
    """Moves drone to target location in local NED frame. Takes vehicle object, north, east, down as parameters.
    North, east, down are in meters. """
    #save initial location
    initialloc = vehicle.location.global_relative_frame
    #move vehicle to target location
    send_local_ned_position(vehicle, north, east, down)
    xyplanar_movement = math.sqrt(north**2 + east**2)
    #checks if xy planar movement is reached and altitude is reached
    while vehicle.mode.name=='GUIDED':
        currentloc = vehicle.location.global_relative_frame
        moved_distance=  get_distance_meters(initialloc, currentloc)
        if moved_distance > xyplanar_movement*0.99:
            print('xy planar movement reached')
            time.sleep(1)
            break
        if currentloc.alt <= initialloc.alt + down*0.99:
            print('altitude reached')
            time.sleep(1)
            break
        time.sleep(1)
    print('Movement finished')
    
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

def send_local_ned_position(vehicle, north, east, down):
    """ Sends position commands to drone, takes vehicle object, north, east, down as parameters.
    Position commands are in NED frame relative to drone. Position commands in meters.
    This msg can be interupted by other commands"""
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,0,0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111111000,
        north,east,down,
        0,0,0,
        0,0,0,
        0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def condition_yaw(vehicle, heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

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
    If aruco marker is not found, vehicle goes to safety_height, sets mode to guided and raises LoiterError exception. (NOT WORKING YET!)"""

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
    vehicle.parameters['PLND_STRICT'] = 2  #2 for strict landing, 1 for loose landing

    #Precision Loiter Logic

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        time.sleep(0.5)
    set_altitude(vehicle, loiter_height)

    found_count = 0
    notfound_count = 0
    search_frames = 0
    abort_counter = 0
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

def SAR_search_pattern(vehicle,
                        x_area, y_area,
                        x_landpad, y_landpad,
                        search_height,
                        horizontal_res, vertical_res,
                        doRTL = False, 
                        debug = False):
    """"This function runs the SAR pattern based on the estimated size of the SAR task field and the estimated start landing/takeoff pad.
    This function returns the Lat/Long of the start point and 2 photos with their asociated Lat/Longs. Photos are saved as OpenCV captures, 
    and also are saved into .../AlicantoFS/SARPhotos/SAR-YEAR-MONTH-DAY-HOUR-MINUTE-SECOND-PhotoNumber.jpg"""

#                                                                                            #(x_area,y_area)
#                ##############################################################################
#                #                                                                            #
#                # ------x_side------                                     -------x_side------ #  #El drone tiene que ser orientado de tal manera
#                #                                                                            #  #que el eje x positivo apunte hacia el este del drone y el                                                                       
#                #                                                                            #  #eje y positivo apunte hacia el norte del drone
#                #                    ----------x_between_photos----------                    #
#                #                                                                            #
#     #y_area    #           Photo1(x1,y_area/2)                Photo2(x2,y_area/2)           #
#                #                                                                            #
#                #--x_landpad--                                                               #
#                #         [   ^x  ]                                                          #
#              ^ #         [   |   ] |                                                        #
#              | #         [   |_ y] |  y_landpad                                             #
#              | #                   |                                                        #
#              | #                   |                                                        #  
#              + ##############################################################################
#              #(0,0)+------>                       #x_area

    #Move vehicle to search height
    ###set_altitude(vehicle, search_height)

    #Save initial yaw orientation relative to global
    #initial_yaw = vehicle.heading

    #Set yaw to hold and orient to current heading
    ###condition_yaw(vehicle, 0, relative=True)
    ###time.sleep(5)
    
    #calculate relative movement neeeded to go to Photo1
    relative_x_photo1 = 0
    relative_y_photo1 = 0

    if y_landpad > y_area/2: # if the landing pad is in the top half of the search area
        relative_y_photo1 = -1 * (y_landpad - y_area/2)
    elif y_landpad < y_area/2: # if the landing pad is in the bottom half of the search area
        relative_y_photo1 = y_area/2 - y_landpad
    else:
        relative_y_photo1 = 0
    
    x_between_photos = 0.35 * x_area
    x_side = (x_area - x_between_photos)/2
    
    if x_landpad > x_side: # if the landing pad is right of Photo1
        relative_x_photo1 = -1 * (x_landpad - x_side)
    elif x_landpad < x_side: # if the landing pad is left of Photo1
        relative_x_photo1 = x_side - x_landpad

    if(debug == True):
        print("Modo Debug, coordenadas son entregadas de manera reliativa al drone ((+)Adelante/(-)Atras, (+)Derecha/(-)Izquierda)")
        print("Movimiento calculado para ir a Photo1 desde LandPad: ",relative_y_photo1, relative_x_photo1)
        print("Movimiento calculado para ir a Photo2 desde Photo1: ",0, x_between_photos)

    #Start OpenCV capture and set resolution 
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error opening video stream from camera")

    cap.set(3, horizontal_res)
    cap.set(4, vertical_res)

    #Set path to save photos
    save_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "SARPhotos")

    #Move to Photo1
    ###goto_local_frame(vehicle, relative_y_photo1, relative_x_photo1, 0)
    ###time.sleep(5)

    #Orient yaw to global north
    #condition_yaw(vehicle, 0, relative=False)
    #time.sleep(3)

    #Take Photo1, save as OpenCV capture and save as .jpg, save lat/long of photo and home point
    success1, photo1 = cap.read()
    if not success1:
        print("Error reading photo1")

    photo1_name = "SAR-" + str(datetime.datetime.now().year) + "-" + str(datetime.datetime.now().month) + "-" + str(datetime.datetime.now().day) \
                 + "-" + str(datetime.datetime.now().hour) + "-" + str(datetime.datetime.now().minute) + "-" + str(datetime.datetime.now().second) + "-Photo1.jpg"
    cv2.imwrite(os.path.join(save_path, photo1_name), photo1)
    photo1_lat = vehicle.location.global_relative_frame.lat
    photo1_long = vehicle.location.global_relative_frame.lon

    #If vehicle has home location save coordinates, else return None for lat/long
    if vehicle.home_location is not None:
        home_lat = vehicle.home_location.lat
        home_long = vehicle.home_location.lon
    else:
        home_lat = None
        home_long = None

    #Reorient yaw to initial orientation
    #condition_yaw(vehicle, initial_yaw, relative=False)
    #time.sleep(3)

    #Move to Photo2
    ###goto_local_frame(vehicle, 0, x_between_photos, 0)
    ###time.sleep(5)

    #Orient yaw to global north
    #condition_yaw(vehicle, 0, relative=False)
    #time.sleep(3)

    #Take Photo2
    success2, photo2 = cap.read()
    if not success2:
        print("Error reading photo2")

    photo2_name = "SAR-" + str(datetime.datetime.now().year) + "-" + str(datetime.datetime.now().month) + "-" + str(datetime.datetime.now().day) \
                    + "-" + str(datetime.datetime.now().hour) + "-" + str(datetime.datetime.now().minute) + "-" + str(datetime.datetime.now().second) + "-Photo2.jpg"
    cv2.imwrite(os.path.join(save_path, photo2_name), photo2)
    photo2_lat = vehicle.location.global_relative_frame.lat
    photo2_long = vehicle.location.global_relative_frame.lon
    
    #Reorient yaw to initial orientation
    #condition_yaw(vehicle, initial_yaw, relative=False)
    #time.sleep(3)

    if(doRTL == True):
        vehicle.mode = VehicleMode("RTL")
        while vehicle.mode != "RTL":
            time.sleep(0.5)
        print("Vehicle is now in RTL mode")
    
    return photo1, photo2, photo1_lat, photo1_long, photo2_lat, photo2_long, home_lat, home_long
    
###########DEPENDENCIES################
from distutils.log import debug
import time
import socket
import math
import argparse
from arm_takeoff_3 import arm, takeoff, connectMyCopter

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
from pymavlink import mavutil

import cv2
import cv2.aruco as aruco
import numpy as np

from imutils.video import WebcamVideoStream
import imutils
#######VARIABLES####################
##Aruco
id_to_find = 72
marker_size = 19 #cm
takeoff_height = 8
velocity = .5
debug_mode = True

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()
##

##Camera
horizontal_res = 800
vertical_res = 600
cap = WebcamVideoStream(src=0, width=horizontal_res, height=vertical_res).start()

horizontal_fov =59  * (math.pi / 180 ) ##Pi cam V1: 53.5 V2: 62.2
vertical_fov =46  * (math.pi / 180)    ##Pi cam V1: 41.41 V2: 48.8

cameraMatrix_rpi   = np.array([[933.9466925521707, 0, 358.59608943398365],
                [0, 935.0635463990791, 293.0721064675127],
                [0, 0, 1]])
cameraDistortion_rpi   = np.array([-0.4530790972005633, 0.3951099938612813, 0.0037673873203789916, 0.0016363264710513889, -0.38177331299300393])
##

##Counters and script triggers
found_count=0
notfound_count=0

first_run=0 #Used to set initial time of function to determine FPS
start_time=0
end_time=0
script_mode = 1 ##1 for arm and takeoff, 2 for manual LOITER to GUIDED land 
ready_to_land=0 ##1 to trigger landing

manualArm=True ##If True, arming from RC controller, If False, arming from this script. 
#########FUNCTIONS#################

def send_local_ned_velocity(vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111,
		0, 0, 0,
		vx, vy, vz,
		0, 0, 0,
		0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()
    
def send_land_message(x,y, debug_setting = False):
    if debug_setting == False:
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
        
    else:
        pass
        print("Vehiculo corrigiendo X: ", x, " Y: ", y)

def lander(debug = False):
    global first_run,notfound_count,found_count,marker_size,start_time
    if first_run==0:
        print("First run of lander!!")
        first_run=1
        start_time=time.time()
        
    frame = cap.read()
    frame = cv2.flip(frame,0)
    frame = cv2.flip(frame,1)
    frame = cv2.resize(frame,(horizontal_res,vertical_res))
    frame_np = np.array(frame)
    gray_img = cv2.cvtColor(frame_np,cv2.COLOR_BGR2GRAY)
    ids=''
    corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)
    if debug == False:
        if vehicle.mode!='LAND':
            vehicle.mode=VehicleMode("LAND")
            while vehicle.mode!='LAND':
                print('WAITING FOR DRONE TO ENTER LAND MODE')
                time.sleep(1)

    try:
        if ids is not None and ids[0] == id_to_find:
            ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=cameraMatrix_rpi,distCoeffs=cameraDistortion_rpi)
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
            x = '{:.2f}'.format(tvec[0])
            y = '{:.2f}'.format(tvec[1])
            z = '{:.2f}'.format(tvec[2])
            
            y_sum = 0
            x_sum = 0
            
            x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
            y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
    
            x_avg = x_sum*.25
            y_avg = y_sum*.25 #relative to OpenCV cordinates

            #if debug == True:
            #    print(f"X Aruco: {x_avg} Y Aruco: {y_avg}")
            
            x_ang = (x_avg - horizontal_res*.5)*(horizontal_fov/horizontal_res)
            y_ang = (y_avg - vertical_res*.5)*(vertical_fov/vertical_res) 
            
            if vehicle.mode!='LAND' and debug == False:
                vehicle.mode = VehicleMode('LAND')
                while vehicle.mode!='LAND':
                    time.sleep(1)
                print("------------------------")
                print("Vehicle now in LAND mode")
                print("------------------------")
                send_land_message(x_ang,y_ang)
            else:
                send_land_message(x_ang,y_ang,debug_setting = debug)
                pass
            #print(("X CENTER PIXEL: "+str(x_avg)+" Y CENTER PIXEL: "+str(y_avg)))
            #print(("FOUND COUNT: "+str(found_count)+" NOTFOUND COUNT: "+str(notfound_count)))
            #print(("MARKER POSITION: x=" +x+" y= "+y+" z="+z))
            found_count=found_count+1
            print("")
        else:
            notfound_count=notfound_count+1
    except Exception as e:
        print(('Target likely not found. Error: '+str(e)))
        notfound_count=notfound_count+1
    
     
 
 
######################################################

#######################MAIN###########################

######################################################

vehicle = connectMyCopter()

##
##SETUP PARAMETERS TO ENABLE PRECISION LANDING
##
vehicle.parameters['PLND_ENABLED'] = 1
vehicle.parameters['PLND_TYPE'] = 1 ##1 for companion computer
vehicle.parameters['PLND_EST_TYPE'] = 0 ##0 for raw sensor, 1 for kalman filter pos estimation
vehicle.parameters['LAND_SPEED'] = 20 ##Descent speed of 30cm/s

if script_mode ==1 and debug_mode == False:
    arm(vehicle)
    takeoff(8,vehicle)
    print((str(time.time())))
    #send_local_ned_velocity(velocity,velocity,0) ##Offset drone from target
    time.sleep(1)
    ready_to_land=1
elif script_mode==2 and debug_mode == False:
    while vehicle.mode!='GUIDED':
        time.sleep(1)
        print(("Waiting for manual change from mode "+str(vehicle.mode)+" to GUIDED"))
    ready_to_land=1

if ready_to_land==1 or debug_mode == True:
    while vehicle.armed==True or debug_mode == True:
        lander(debug = debug_mode)
    end_time=time.time()
    total_time=end_time-start_time
    total_time=abs(int(total_time))

    total_count=found_count+notfound_count
    freq_lander=total_count/total_time
    print(("Total iterations: "+str(total_count)))
    print(("Total seconds: "+str(total_time)))
    print("------------------")
    print(("lander function had frequency of: "+str(freq_lander)))
    print("------------------")
    print("Vehicle has landed")
    print("------------------")
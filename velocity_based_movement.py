from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import exceptions
import math
import argparse
from pymavlink import mavutil

def connectMyCopter():
	parser = argparse.ArgumentParser(description="commands")
	parser.add_argument("--connect")
	args = parser.parse_args()
	
	connection_string =args.connect

	if not connection_string:
		import dronekit_sitl
		sitl= dronekit_sitl.start_default()
		connection_string = sitl.connection_string()


	vehicle = connect(connection_string, wait_ready =True)

	return vehicle

def arm_and_takeoff(targetHeight):
	while vehicle.is_armable !=True:
		print("waiting for vehicle")
		time.sleep(1)

	print("vehicle is now armable")

	vehicle.mode = VehicleMode("GUIDED")

	while vehicle.mode !='GUIDED':
		print("waiting guided mode, curret mode:", vehicle.mode.name)
		time.sleep(1)
	print('mode: guided')

	vehicle.armed = 'True'
	while vehicle.armed !=True:
		print ('waiting arming')
		time.sleep(1)

	print('armed')

	vehicle.simple_takeoff(targetHeight) #meters
	
	while True:
		print("Current altitude: %d"%vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt >= .95*targetHeight:
			break
		time.sleep(1)
	print("target reached")
	return None

def send_local_ned_velocity(vx,vy,vz):
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
	#python mode_setter.py

def send_global_ned_velocity(vx,vy,vz):
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
	#python mode_setter.py





## python connection_template.py --connect 127.0.0.1:14550

vehicle =connectMyCopter()
arm_and_takeoff(10)



counter=0
while counter <5:
	send_local_ned_velocity(5,0,0)
	time.sleep(1)
	print('Moving North Relative to drone')
	counter += 1
time.sleep(2)


counter=0
while counter <5:
	send_local_ned_velocity(-5,0,0)
	time.sleep(1)
	print('Moving South Relative to drone')
	counter += 1
time.sleep(2)

counter=0
while counter <5:
	send_global_ned_velocity(5,0,0)
	time.sleep(1)
	print('Moving to true North')
	counter += 1
time.sleep(2)

while True:
	time.sleep(1)





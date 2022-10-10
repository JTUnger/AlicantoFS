from curses import baudrate
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket

import math
import argparse

def connectMyCopter():
	parser = argparse.ArgumentParser(description="commands")
	parser.add_argument("--connect")
	args = parser.parse_args()
	
	connection_string =args.connect

	baudrate=57600


	vehicle = connect(connection_string,baud=baudrate, wait_ready =True)

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
		
def get_distance_meters(targetloc, currentloc):
	dLat= targetloc.lat - currentloc.lat
	dLon= targetloc.lon - currentloc.lon
	return math.sqrt((dLon*dLon) + (dLat*dLat))*1.113195e5

def goto(targetloc):
	targetdist= get_distance_meters(targetloc, vehicle.location.global_relative_frame)
	
	vehicle.simple_goto(targetloc)
	while vehicle.mode.name=='GUIDED':
		currentdist=  get_distance_meters(targetloc, vehicle.location.global_relative_frame)
		if currentdist < targetdist*0.01:
			print('target reached')
			time.sleep(2)
			break
		time.sleep(1)
	return None
	
	
if __name__ == "__main__":
	#MAIn EXECUTABLE

	wp1 = LocationGlobalRelative(-33.2779604, -70.6298053,13)

	vehicle =connectMyCopter()
	arm_and_takeoff(10)

	#goto(wp1)
	#time.sleep(5)
	#vehicle.mode ='RTL'

	#while vehicle.mode != 'RTL':
	#	print('waiting for RTL mode')
	#	time.sleep(1)
	#print('RTL mode activated')

	while True:
		time.sleep(1)




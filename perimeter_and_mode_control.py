from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import exceptions
import math
import argparse
from threading import Thread, Event
import numpy as np
from scipy.spatial import ConvexHull


def connectMyCopter():
	parser = argparse.ArgumentParser(description="commands")
	parser.add_argument("--connect")
	args = parser.parse_args()
	
	connection_string =args.connect


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
		if ev.is_set():												#
			print('Fuera de Perimetro: Mision Abandonada')			#
			break													#
		time.sleep(1)
		if ev2.is_set():
			print('Modo Abandonado')
			break

	return None


### Nuevas Funciones

def point_in_hull(point, hull, tolerance=1e-12):
    return all(
        (np.dot(eq[:-1], point) + eq[-1] <= tolerance)
        for eq in hull.equations)


def per_check(points):
    hull = ConvexHull(points)
    while True:
        time.sleep(1)
        check = point_in_hull((vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon), hull)
        if check == False:
            ev.set()
            break


def mode_check():                 
    while True:
        if vehicle.mode != 'GUIDED':
            print('Interrupcion')
            ev2.set()
            current_lat = vehicle.location.global_relative_frame.lat
            current_lon = vehicle.location.global_relative_frame.lon
            vehicle.mode = 'GUIDED'
            new_wp = LocationGlobalRelative(current_lat, current_lon, 25)
            vehicle.simple_goto(new_wp)
            break

def cambio_modo():
	time.sleep(10)
	vehicle.mode = 'LOITER'



	
	


#MAIn EXECUTABLE


## python connection_template.py --connect 127.0.0.1:14550

pp1 = (-33.110904,-70.778185)
pp2 = (-33.111119,-70.777732)
pp3= (-33.111249,-70.777917)
pp4= (-33.111073,-70.778349)



wp1 = LocationGlobalRelative(-33.111429, -70.778409, 10)
vehicle =connectMyCopter()
arm_and_takeoff(10)
ptos = np.asarray([pp1,pp2,pp3])
ev = Event()
ev2 = Event()
new_thread1 = Thread(target = per_check, args = (ptos,))
time.sleep(1)
#new_thread2 = Thread(target = mode_check)
#new_thread3 = Thread(target = cambio_modo)
new_thread1.start()
#new_thread2.start()
#new_thread3.start()
goto(wp1)

#vehicle.mode ='LAND'


#while vehicle.mode != 'LAND':
#	print('waiting for land mode')
#	time.sleep(1)
#print('LAND mode activated')

while True:
	time.sleep(1)




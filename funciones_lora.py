

import serial
import time
import random
import struct
from dronekit import *

def send_and_recieve_data(mensaje):

	if ser.in_waiting>0:
		#recieve
		read_serial=ser.readline().decode('utf-8')
		ser.flush()
		print(read_serial)
		

		#send
		ser.write(mensaje.encode('utf-8'))
		ser.flush()
		time.sleep(0.2)

def reportar_status_SAR(vehicle):
	"""Lee el status del drone y lo transforma en un mensaje que indica el status del drone. Se usa para el task de SAR.
	Las opciones son: Manual, Autonomous, Faulted. Toma de entrada un objeto Vehicle y de salida retorna un string que indica
	el status, se usa dentro de un loop esta funcion."""
	if vehicle.system_status == "EMERGENCY" or vehicle.system_status == "CRITICAL":
		return "Faulted"

	if vehicle.mode.name == "GUIDED" or vehicle.mode.name == "RTL" or vehicle.mode.name == "LAND":
		return "Autonomous"

	elif vehicle.mode.name == "STABILIZE" or vehicle.mode.name == "POSHOLD":
		return "Manual"

	else:
		return "Faulted"

def reportar_status_HEARTBEAT(vehicle):
	"""Lee el status del drone y lo transforma en un mensaje que indica el status del drone. Se usa para el heartbeat.
	Las opciones son: Stowed, Deployed, Faulted. Toma de entrada un objeto Vehicle y de salida retorna un string que indica
	el status, se usa dentro de un loop esta funcion."""
	if vehicle.system_status == "EMERGENCY" or vehicle.system_status == "CRITICAL":
		return "Faulted"
	
	elif vehicle.armed == True:
		return "Deployed"

	elif vehicle.armed == False:
		return "Stowed"
	
	else:
		return "Faulted"
	


################### Ejecutable ############################

ser = serial.Serial('/dev/ttyACM0',9600)
ser.flush()

while True:
	
	lat = round(random.uniform(35, 36), 8)
	lon = round(random.uniform(35, 36), 8)
	stri = "Latitud: "+str(lat) + "; Longitud: "+ str(lon) 
	send_and_recieve_data(stri)
	#send_data(time.time())




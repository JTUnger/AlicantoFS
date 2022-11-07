

import serial
import time
import random
import struct

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



################### Ejecutable ############################

ser = serial.Serial('/dev/ttyACM0',9600)
ser.flush()

while True:
	
	lat = round(random.uniform(35, 36), 8)
	lon = round(random.uniform(35, 36), 8)
	stri = "Latitud: "+str(lat) + "; Longitud: "+ str(lon) 
	send_and_recieve_data(stri)
	#send_data(time.time())




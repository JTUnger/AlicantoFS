from flight_functions import *
from cv_functions import *
#from funciones_lora import *
import exceptions
import time 

#Task parameters and field dimensions
takeoff_height = 4
x_area = 5
y_area = 5
x_landpad = 1
y_landpad = 1
search_height = 6
horizontal_res = 1920
vertical_res = 1080
doRTL = False
debug = True

#Connect to vehicle and return Vehicle object
vehicle = connectMyCopter()

#Arm vehicle and takeoff to specified height
arm_takeoff(vehicle, takeoff_height)

#Run SAR search pattern and grab images and GPS coordinates (Se que se ve horrible, pero es lo que hay)
(photo1, photo2,
photo1_lat, photo1_long, 
photo2_lat, photo2_long, 
home_lat, home_long) = SAR_search_pattern(vehicle, x_area, y_area, x_landpad, y_landpad, 
                                            search_height, horizontal_res, vertical_res, 
                                                doRTL, debug)

#Use OpenCV to process images and return the coordinates of the target
pass
print("END")

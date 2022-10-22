from flight_functions import *
from exceptions import *
import time 

vehicle = connectMyCopter()
arm_takeoff(vehicle, 4)
aruco_precision_landing(vehicle, start_height=5, safety_height= 10)

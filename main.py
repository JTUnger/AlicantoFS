from flight_functions import *
from exceptions import *
import time 

vehicle = connectMyCopter()
arm_takeoff(vehicle, 4)
aruco_precision_landing(vehicle, 8, safety_height= 12)

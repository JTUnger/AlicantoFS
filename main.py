from flight_functions import *
from exceptions import *
import time 

vehicle = connectMyCopter()
arm_takeoff(vehicle, 4)
loiter_aruco(vehicle, 7, 20)

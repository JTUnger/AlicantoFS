from flight_functions import *
from exceptions import *
import time 

vehicle = connectMyCopter()
arm_takeoff(vehicle, 5)
time.sleep(3)
set_altitude(vehicle, 8)
from flight_functions import *

vehicle = connectMyCopter()
arm_takeoff(vehicle, 5)
goto_local_frame(vehicle, 1, 0, 0)
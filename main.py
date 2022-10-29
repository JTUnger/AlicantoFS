from flight_functions import *
from exceptions import *
import time 

vehicle = connectMyCopter()
time.sleep(2)
play_tune(vehicle,"AAAA")
from flight_functions import *
from cv_functions import *
#from funciones_lora import *
import exceptions
import time 
from threading import Thread
from serial import Serial
from time import sleep

class SarControl():
    def __init__(self, port="/dev/ttyACM0", baud=9600) -> None:
        self.PORT = port
        self.BAUD = baud
        self.ser_samd21 = None
        self.takeoff_height = 4
        self.x_area = 6
        self.y_area = 6
        self.x_landpad = 1
        self.y_landpad = 1
        self.search_height = 6
        self.horizontal_res = 1920
        self.vertical_res = 1080
        self.doRTL = False
        self.debug = True
        self.done = False
        self.heart_data = {
            "status": 1,
            "objectA": None,
            "latA": None,
            "nsA": None,
            "lonA": None,
            "ewA": None,
            "objectB": None,
            "latB": None,
            "nsB": None,
            "lonB": None,
            "ewB": None,
            "id": "CALCH"  # TODO: verificar id equipo
        }
        self.status_values = {"Manual": 1, "Autonomous": 2, "Faulted": 3}
        self.status_format = ["objectA", "latA", "nsA", "lonA", "ewA", "objectB", "latB", "nsB", "lonB", "ewB", "id", "status", ]
        self.vehicle = None
        self.heart_thread = Thread(target=self.heartbeat)
    
    def heartbeat(self) -> None:
        def parse_heart(data: dict) -> str:
            out = ""
            for val in self.status_format:
                if data[val] is not None:
                    out += f"{data[val]},"
                else:
                    out += ","
        def transmit_heart(data: str) -> None:
            self.ser_samd21.write(data)
        while not self.done:
            out = parse_heart(self.heart_data)
            transmit_heart(out)
            sleep(1)


    def run_sar(self) -> None: 
        self.ser_samd21 = Serial(self.PORT, self.BAUD)
        self.vehicle = connectMyCopter()
        self.heart_thread.start()
        arm_takeoff(self.vehicle, self.takeoff_height)
        sar_out = SAR_search_pattern(self.vehicle, self.x_area, self.y_area, self.x_landpad, self.y_landpad,
                                                    self.search_height, self.horizontal_res, self.vertical_res,
                                                        self.doRTL, self.debug)
        # TODO: revisar formato de SAR_search_pattern

        self.heart_thread.join()
        self.done = True
        self.ser_samd21.close()
        print("END OF SAR")	

if __name__ == "__main__":
    sar = SarControl()
    sar.run_sar()
#!/usr/bin/env python3

# source ./sar_ws/devel/setup.bash
# sudo chmod a+rw /dev/ttyUSB0

import os
import math
import rospy
import tf
from std_msgs.msg import Float32MultiArray, Int16
from serial import Serial
from threading import Thread

class PolarSub():
    def __init__(self, port: str, baud: int) -> None:
        self.PORT = port
        self.BAUD = baud
        self.lat = float()
        self.lon = float()
        self.lora_ser = Serial(self.PORT, self.BAUD)
        self.pol_sub = rospy.Subscriber("/nav/polar", Float32MultiArray, self.polar_callback)
        self.stt_pub = rospy.Publisher("/uav/status", Int16, queue_size=60)
        self.status = {1: "Stowed", 2: "Deployed", 3: "Faulted"}
        self.loop_thread = Thread(target=self.loop)
        self.loop_thread.start()
    
    def build_data(self) -> list:
        out = f"{self.lat},{self.lon}"
        return out

    def polar_callback(self, data: Float32MultiArray) -> None:
        self.lat = out[0]
        self.lon = out[1]
        out = self.build_data()
        self.lora_ser.write(out)
    
    def parse_data(self, data: str) -> int:
        out = 3
        data = data.split(",")
        if data[0] in self.status.keys():
            # string o int?? Cual es el indice del status??
            out['status'] = data[0]
        return out

    def loop(self) -> None:
        print("Starting LoRa loop!")
        while not rospy.is_shutdown():
            info = self.lora_ser.read()
            parsed_data = self.parse_data(info)
            self.stt_pub.publish(parsed_data)

if __name__ == "__main__":
    print("Starting SAR LoRa sub!")
    PORT = '/dev/ttyUSB0'
    BAUD = 9600
    print(f"Attempting to setup LoRa transceiver at {PORT}:{BAUD}")
    pol_node = PolarSub(port=PORT, baud=BAUD)
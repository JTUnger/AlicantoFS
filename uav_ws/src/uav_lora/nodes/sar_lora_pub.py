#!/usr/bin/env python3

# source ./sar_ws/devel/setup.bash
# sudo chmod a+rw /dev/ttyUSB0

import os
import math
import rospy
import tf
from std_msgs.msg import Float32MultiArray
from serial import Serial

class UavRos():
    def __init__(self, port: str, baud: int) -> None:
        self.PORT = port
        self.BAUD = baud
        self.dat_pub = rospy.Publisher("/uav/data", list, queue_size=60)
        self.stt_pub = rospy.Publisher("/uav/status", int, queue_size=60)
        self.status = {1: "Manual", 2: "Autonomous", 3: "Faulted"}
    
    def build_data(self, data: str) -> list:
        out = data.split(",")
        return out
    
    def build_status(self, status: int) -> list:
        pass
    
    def loop(self) -> None:
        while not rospy.is_shutdown():
            info = self.lora_ser.read()
            out = self.build_data(info)
            self.dat_pub.publish(out)

    
    def sar_init(self) -> None:
        self.lora_ser = Serial(self.PORT, self.BAUD)
        self.loop()

if __name__ == "__main__":
    print("Starting SAR LoRa pub!")
    PORT = '/dev/ttyUSB0'
    BAUD = 9600
    print(f"Attempting to setup LoRa transceiver at {PORT}:{BAUD}")
    sar_node = SarRos(port=PORT, baud=BAUD)
    sar_node.sar_init()
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
    
    def parse_data(self, data: str) -> dict:
        data = data.split(",")
        out = {'data': None, 'status': None}
        if data[0] in self.status.keys():
            # string o int?? Cual es el indice del status??
            out['status'] = data[0]
        else:
            out['data'] = ','.join(data)
            return out
        _, *data = data
        out['data'] = ','.join(data)
        return out
    
    def loop(self) -> None:
        while not rospy.is_shutdown():
            info = self.lora_ser.read()
            parsed_data = self.parse_data(info)
            if parsed_data['data']:
                self.dat_pub.publish(parsed_data['data'])
            if parsed_data['status']:
                self.stt_pub.publish(parsed_data['status'])
    
    def sar_init(self) -> None:
        self.lora_ser = Serial(self.PORT, self.BAUD)
        self.loop()

if __name__ == "__main__":
    print("Starting SAR LoRa pub!")
    PORT = '/dev/ttyUSB0'
    BAUD = 9600
    print(f"Attempting to setup LoRa transceiver at {PORT}:{BAUD}")
    sar_node = UavRos(port=PORT, baud=BAUD)
    sar_node.sar_init()
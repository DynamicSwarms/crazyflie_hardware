#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from cflib.utils import uri_helper
from cflib.crtp.crtpstack import CRTPPacket

import cflib.drivers
import cflib.crtp

from crtp_driver.IsseCrazyflie import IsseCrazyflie


import time
import struct
from threading import Thread
import keyboard
ping = CRTPPacket(0xFF)
import array


from cflib.drivers import crazyradio
import logging
from std_srvs.srv import Trigger
def keyboard_thread(cf):
    pass
   # while not keyboard.is_pressed('a'): 
   #     print(cf.state)
   #     keyboard.wait('f')
   #     cf.send_packet(ping)


from crtp_interface.msg import CrtpPacket
from crtp_interface.msg import CrtpResponse
from crtp_interface.msg import SetAutoping
from crtp_interface.srv import CrtpPacketSend


class ResponseListener(Node):
    def __init__(self):
        super().__init__("listener")
        self.create_subscription(CrtpResponse, "crazyradio/crtp_response",self.handle_response,  10)
    def __del__(self):
        self.destroy_node()

    def handle_response(self, response):
        data = response.packet.data
        data_length = response.packet.data_length
        channel = response.packet.channel
        port = response.packet.port
        string = "[" + str(port) + ":" + str(channel) + "] "
        if port == 0 and channel == 0 and data_length: 
            for i in range(data_length):
                string = string + chr(data[i])
        elif port == 2 and channel == 0 and data_length: ## log to ask
            string = string + "see cf"
            #for i in range(data_length):
            #    if data[i] >= 65 and data[i] <= 122: #only allow chars
            #        string = string + chr(data[i])
            #    else:
            #        string = string + '{:02x} '.format(data[i]) 
        elif port == 5 and channel == 2:
            if len(data) > 15:
                string = string + str(struct.unpack('<BBBfff', bytearray(data[1:16])))
            else: string = string  + "Wrong datalength"
        else:
            string = string + "[" +  ''.join('{:02x} '.format(x) for x in data[:data_length])    + "]"
        
        if port == 15 and channel == 3: return
        self.get_logger().info(string)
        #self.get_logger().info(str(ack))

def main():
    rclpy.init()
    listener = ResponseListener()
    rclpy.spin(listener)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
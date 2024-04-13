#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


import time
from threading import Thread
from threading import Semaphore
import array


from cflib.drivers import crazyradio
import logging

from crtp_interface.msg import CrtpPacket
from crtp_interface.msg import CrtpResponse
from crtp_interface.msg import SetAutoping
from crtp_interface.srv import CrtpPacketSend

def start_logging():
      logging.basicConfig(
            format='%(asctime)s.%(msecs)03d %(levelname)-8s %(message)s',
            level=logging.DEBUG,
            datefmt='%Y-%m-%d %H:%M:%S')

class Autoping(Thread): 
    ping = array.array('B')
    ping.append(0xFF)
    def __init__(self, send_cb, rate):
        self.send_cb = send_cb
        self.rate = rate
        
        self.stop = False
        self.start()

    def run(self):
        while not self.stop and not self.rate == 0:
            self.send_cb(ping)
            time.sleep(1/self.rate)
    def set_rate(self, rate):
        self.rate = rate

class Crazyradio(Node):
    def __init__(self):
        super().__init__('crazyradio')
        start_logging()
        ## some ros arg for device_id
        radio_id = 0
        self.color = 0

        #self.radio = crazyradio.Crazyradio(devid=radio_id)
        #self.radio_semaphore = Semaphore(1)

        self.autopings = {}
        self.create_service(CrtpPacketSend, "crazyradio/send_crtp_packet", self.send_crtp_packet)

        self.create_subscription(SetAutoping, "crazyradio/set_autoping", self.set_autoping,10)
        self.crtp_response = self.create_publisher(CrtpResponse, "crazyradio/crtp_response", 10)
      
       
       
        #cflib.crtp.init_drivers()
        #self.links = {}
        #self.test_routine()

       
    def __del__(self):
        self.destroy_node()
        self.radio.close()

    def send_packet(self, channel, address, datarate, data):
        self.get_logger().info("Sending:" + str(channel) + "/" + str(address) + "/" + str(datarate) + "/" + str(data))
    
    def send_crtp_packet(self, msg, response):
        pk = msg.packet
        data = array.array('B')
        data.append(pk.port << 4 | pk.channel)
        for i in range(pk.data_length): 
            data.append(pk.data[i])
        self.send_packet(msg.channel, tuple(msg.address.tolist()), msg.datarate, data)
        return response

    def set_autoping(self, msg):
        ident = (msg.channel, msg.address, msg.datarate)
        if ident in self.autopings:
            self.autopings[ident].set_rate(msg.rate)
        else: 
            self.autopings[ident] = Autoping(msg.channel, msg.address, msg.datarate)

def main():
    rclpy.init()
    radio = Crazyradio()
    rclpy.spin(radio)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
   
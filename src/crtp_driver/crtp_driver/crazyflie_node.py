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

from std_msgs.msg import Int16

from crtp_driver.HighLevelCommander import HighLevelCommander

class Crazyflie(Node):
    COMMAND_TAKEOFF = 7
    COMMAND_LAND = 8
    SETPOINT_HL = 0x08
    def __init__(self):
        super().__init__("listener")
        self.channel = 101
        self.address = (0xE7, 0xE7,0xE7,0xE7, 0x10)
        self.datarate = 2

        self.cf_prefix = 'cf16'

        self.hl_commander = HighLevelCommander()

        self.send_packet_service = self.create_client(CrtpPacketSend, "crazyradio/send_crtp_packet")
        while not self.send_packet_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Send CRTP Packet Service not available, waiting again...")


        self.create_subscription(Int16, self.cf_prefix + "/set_color", self.set_color,  10)
        self.create_subscription(Int16, self.cf_prefix + "/takeoff", self.takeoff, 10)
        self.create_subscription(Int16, self.cf_prefix + "/land", self.land, 10)

        self.create_subscription(Int16, self.cf_prefix + "/set_group_mask", self.set_group_mask, 10)


    def __del__(self):
        self.destroy_node()
    
    def _prepare_send_request(self):
        req = CrtpPacketSend.Request()
        req.channel = self.channel
        req.address = self.address
        req.datarate = self.datarate
        return req

    def set_group_mask(self, msg):
        group = msg.data
        req = self._prepare_send_request()
        req.packet = self.hl_commander.set_group_mask(group)
        self.send_packet_service.call_async(req)


    def land(self, msg):
        group_mask = 0
        absolute_height_m = 0.0
        yaw=0.0
        duration_s = 5.0
        req = self._prepare_send_request()
        req.packet = self.hl_commander.land(absolute_height_m, duration_s, group_mask, yaw)
        self.send_packet_service.call_async(req)

    def takeoff(self, msg):
        group_mask = 0
        absolute_height_m = 1.0
        yaw = 0
        duration_s = 5.0
        req = self._prepare_send_request()
        req.packet = self.hl_commander.takeoff(absolute_height_m, duration_s, group_mask, yaw)
        self.send_packet_service.call_async(req)


    def set_color(self, msg):
        color = msg.data
        pk = CrtpPacketSend.Request()
        pk.channel = self.channel
        pk.address = self.address
        pk.datarate = self.datarate
        pk.packet.port = 2
        pk.packet.channel = 2
        pk.packet.data[0] = 0x16
        pk.packet.data[1] = 0x00
        pk.packet.data[2] = color
        pk.packet.data_length = 3
        self.send_packet_service.call_async(pk)



def main():
    rclpy.init()
    cf = Crazyflie()
    rclpy.spin(cf)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
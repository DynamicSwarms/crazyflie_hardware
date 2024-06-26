#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from cflib.utils import uri_helper
from cflib.crtp.crtpstack import CRTPPacket

import cflib.drivers
import cflib.crtp

from crtp_driver.IsseCrazyflie import IsseCrazyflie


import time
import struct
from threading import Thread
from threading import Semaphore
import keyboard
ping = CRTPPacket(0xFF)
import array


from cflib.drivers import crazyradio
import logging
from std_srvs.srv import Trigger

import time

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
        ## some ros arg for device_id
        radio_id = 0
        self.color = 0

        self.radio = crazyradio.Crazyradio(devid=radio_id)
        self.radio_semaphore = Semaphore(1)

        self.autopings = {}
        self.create_service(Trigger, "crazyradio/send_packet_default", self.send_packet_default)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history= QoSHistoryPolicy.KEEP_ALL,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.create_service(CrtpPacketSend, "crazyradio/send_crtp_packet", self.send_crtp_packet, qos_profile =qos_profile )

        self.create_subscription(SetAutoping, "crazyradio/set_autoping", self.set_autoping,10)

        self.crtp_response = self.create_publisher(CrtpResponse, "crazyradio/crtp_response", qos_profile=qos_profile)
        logging.basicConfig(
            format='%(asctime)s.%(msecs)03d %(levelname)-8s %(message)s',
            level=logging.DEBUG,
            datefmt='%Y-%m-%d %H:%M:%S')
       
       
        #cflib.crtp.init_drivers()
        #self.links = {}
        #self.test_routine()

       
    def __del__(self):
        self.destroy_node()
        self.radio.close()

    def send_packet(self, channel, address, datarate, data):
        self.radio_semaphore.acquire()
        self.radio.set_channel(channel)
        self.radio.set_address(address)
        self.radio.set_data_rate(datarate)
        self.get_logger().info(str(data))
        ack, orack = self.radio.send_packet(data) 
        #self.get_logger().info(str(orack))
         
        self.radio_semaphore.release()
        if ack is None or ack.ack is False or not len(ack.data) > 0:
            self.get_logger().info("No acknowledgement")
            self.get_logger().warn(str(ack.ack) +  str(ack.powerDet) + str(ack.retry) + str(ack.data))
            return 
        response = CrtpResponse()
        response.channel = channel
        response.address = address
        response.packet.port = (ack.data[0] & 0b11111100) >> 4
        response.packet.channel = ack.data[0] & 0b11
        for i, byte in enumerate(ack.data[1:]):
            response.packet.data[i] = byte
        response.packet.data_length = len(ack.data[1:])
        self.crtp_response.publish(response)
    
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
















    
    
    def send_packet_default(self, empty, response):
        self.color = self.color + 1
        if (self.color > 9): self.color = 0
        channel = 101
        address = [0xE7, 0xE7, 0xE7, 0xE7, 0x10 ]
        address = tuple(address)
        #address = (0xE7, 0xE7,0xE7,0xE7, 0x10)
        #address = (0xFF, 0xE7,0xE7,0xE7, 0xE7) ## for broadcasting send 0xFF - E7*s
        
        datarate = 2
        data = array.array('B')
        data.append(0x22) # channel 2 port 2
        data.append(0x16)
        data.append(0x00)
        data.append(self.color)
        #data.append(0xFF)
        self.send_packet(channel, address, datarate, data)
        return response

    def print_log(self, info):
        self.get_logger().info(str(info))
        if len(info.data) < 8: return 
        value = struct.unpack('<f', info.data[4:])[0]
        self.get_logger().info(str(value))

    def print_console(self, info):
        string = ""
        for c in info.data:
            string = string + chr(c)
        self.get_logger().info(string)
        
    def test_routine(self):
        self.get_logger().info("Hello World")
        uri = uri_helper.uri_from_env(default='isseradio://0/101/2M/E7E7E7E710')
        self.get_logger().info(uri)
        cf = IsseCrazyflie()
        cf.add_port_callback(5, self.print_log)
        cf.add_port_callback(0, self.print_console)


        Thread(target=keyboard_thread, args=(cf,)).start()
        cf.open_link(uri)
        packet = [0xf3, 0xfe,0xFF ] #reset init
        cf.send_non_crtp(packet)
        time.sleep(2)
        packet = [0xf3, 0xfe, 0xF0,1] # reset
        cf.send_non_crtp(packet)
        time.sleep(4)

        for i in range(70): cf.send_packet(ping) ## catch all beginning data
        time.sleep(2)
        for i in range(10): cf.send_packet(ping)
        self.get_logger().info("Done init")

        pk= CRTPPacket()
        pk.set_header(5,0) #logging info
        pk.data = (3,0) # get info
        cf.send_packet(pk)
        self.get_logger().info("requested info") 
        time.sleep(0.2)

        pk = CRTPPacket()
        pk.set_header(5, 1) # 5 logging, 1 setting
        pk.data = (6, 0xa) # command create block is v1/2 6 0
        pk.data.append(0x77)
        pk.data.append(0x53)
        pk.data.append(0x01)

        #pk.data.append(0x07)          # 0x01 uint8_t , 7 float 
        #pk.data.append(339 & 0x0ff)
        #pk.data.append((339 >>8) & 0x0ff)
        #self.get_logger().info('Adding/appending log block id {}'.format(1))
        cf.send_packet(pk)
        self.get_logger().info("Create" + str(pk))

        time.sleep(1)

        pk = CRTPPacket()
        pk.set_header(5, 1) # CHAN_SETTINGS
        pk.data = (3, 0xa, 10) # startlogging, id= 0, period= (factor of 10)
        cf.send_packet(pk)
        self.get_logger().info("Start" + str(pk))
        cf.send_packet(ping)

        seconds = 2
        for i in range(100 * seconds): 
            cf.send_packet(ping)
            time.sleep(0.01)

        ##it logs something but not the correct thihng

        time.sleep(5)
        cf.close_link()
        self.get_logger().info("Done")

def main():
    rclpy.init()
    radio = Crazyradio()
    rclpy.spin(radio)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import  ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from cflib.utils import uri_helper
from cflib.crtp.crtpstack import CRTPPacket

import cflib.drivers
import cflib.crtp

from crtp_driver.IsseCrazyflie import IsseCrazyflie


import time
import struct
from threading import Thread, Event, Lock
from threading import Semaphore
import keyboard
ping = CRTPPacket(0xFF)
import array

from crtp_driver.connection_watchdog import ConnectionWatchdog

from cflib.drivers import crazyradio
import logging
from std_srvs.srv import Trigger

import time

from queue import Queue

from .radioQueue import Link, RadioPacket, RadioQueue

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

class OutPacket():
    def __init__(self, channel, address, datarate, data, response_bytes):
        self.channel = channel
        self.address= address
        self.datarate = datarate
        self.data = data
        self.response_bytes = response_bytes

class Crazyradio(Node):
    def __init__(self):
        super().__init__('crazyradio')
        ## some ros arg for device_id
        radio_id = 0
        self.color = 0

        self.radio = crazyradio.Crazyradio(devid=radio_id)
        self.radio_semaphore = Semaphore(1)

        self.watchdog = ConnectionWatchdog(self.send_packet)
        self.watchdog_timer = self.create_timer(callback=self.watchdog.resend_lost_packages, timer_period_sec=0.01) # 10 ms

        self.autopings = {}


        callback_group = ReentrantCallbackGroup()

        self.create_service(Trigger, "crazyradio/send_packet_default", self.send_packet_default, callback_group=callback_group)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history= QoSHistoryPolicy.KEEP_ALL,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.create_service(CrtpPacketSend, "/crazyradio/send_crtp_packet", self.send_crtp_packet, qos_profile =qos_profile, callback_group=callback_group )

        self.create_subscription(SetAutoping, "crazyradio/set_autoping", self.set_autoping,10)

        self.crtp_response = self.create_publisher(CrtpResponse, "crazyradio/crtp_response", qos_profile=qos_profile, callback_group=callback_group)
        logging.basicConfig(
            format='%(asctime)s.%(msecs)03d %(levelname)-8s %(message)s',
            level=logging.DEBUG,
            datefmt='%Y-%m-%d %H:%M:%S')
       
       
        #cflib.crtp.init_drivers()
        #self.links = {}
        #self.test_routine()

        self.out_queue = Queue()
        self.in_queue = Queue()

        self.create_timer(0.01, self.sendRoutine, callback_group=None) ## 100 Hz for now

        self.radioQueue = RadioQueue()
        self.radioQueueSemaphore = Semaphore(1)
       
    def __del__(self):
        self.destroy_node()
        self.radio.close()

    def setup_radio(self, channel, address, datarate):
        self.radio.set_channel(channel)
        self.radio.set_address(address)
        self.radio.set_data_rate(datarate)
        
    def sendRoutine(self):
        self.radioQueueSemaphore.acquire()
        link, data = self.radioQueue.getRadioPacket()
        self.radioQueueSemaphore.release()
        if not data: return 
        self.get_logger().info("send")

        respData = self.send_packet(link.channel, link.address, link.datarate, data)
        if respData:
            self.radioQueueSemaphore.acquire()
            link.handle_response(respData)
            self.radioQueueSemaphore.release()
            self.get_logger().info("sendend")


    def send_packet(self, channel, address, datarate, data):
        self.get_logger().info(str(data))
        self.radio_semaphore.acquire()
        self.setup_radio(channel, address, datarate)
        ack = self.radio.send_packet(data) 
        self.radio_semaphore.release()


        if ack is None or ack.ack is False:
            self.get_logger().info("No acknowledgement")
            #self.get_logger().warn(str(ack.ack) +  str(ack.powerDet) + str(ack.retry) + str(ack.data))
            return None
        if not len(ack.data) > 0:
            self.get_logger().info("Empty Response: FirmwareIssue: #703")
            return None
        response = CrtpResponse()
        response.channel = channel
        response.address = address
        response.packet.port = (ack.data[0] & 0b11111100) >> 4
        response.packet.channel = ack.data[0] & 0b11
        for i, byte in enumerate(ack.data[1:]):
            response.packet.data[i] = byte
        response.packet.data_length = len(ack.data[1:])
        #self.watchdog.free_packet(channel, address, datarate, ack.data)
        self.crtp_response.publish(response)
   
        return ack.data

    def send_crtp_packet(self, msg, response):
        self.get_logger().info("Enqueuing")

        pk = msg.packet
        data = array.array('B')
        data.append(pk.port << 4 | pk.channel)
        for i in range(pk.data_length): 
            data.append(pk.data[i])

        if not msg.response_bytes:
            self.send_packet(msg.channel, tuple(msg.address.tolist()), msg.datarate, data)
            return response 

        l = Link(msg.channel,tuple(msg.address.tolist()), msg.datarate)
        pk = RadioPacket(data, msg.response_bytes)

        event=Event()
        resp = []
        self.radioQueueSemaphore.acquire()
        self.radioQueue.addRadioPacket(l, pk, event)
        self.radioQueueSemaphore.release()
        

        def done_callback(future):
            nonlocal event
            event.set()

        future = Future() #self.client.call_async(request)
        future.add_done_callback(done_callback)

        # Wait for action to be done
        # self.service_done_event.wait()
        #event.wait()

        return response

        #lck = Lock()
        #lck.acquire()
        #lck.acquire()
    
    
        self.get_logger().info("Passed lock")
        
        #evnt.wait(10) ## wait at most 10 sec

        #success = evt.wait() ## Wait at least one second for the response 
        #if success:
        #    self.get_logger().info("Responded with: " + str(pk.response))
        #else: 
        #    pass

        return response 

        #if msg.response_bytes:
        #    self.watchdog.add_packet_to_guard(msg.channel, tuple(msg.address.tolist()), msg.datarate, data, msg.response_bytes)
        #self.send_packet(msg.channel, tuple(msg.address.tolist()), msg.datarate, data)
        #return response

    def set_autoping(self, msg):
        self.watchdog.log_lost_packets()
        return 
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
    executor = MultiThreadedExecutor()
    rclpy.spin(radio, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
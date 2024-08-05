#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


import logging

from crtp_interface.srv import CrtpPacketSend

from std_msgs.msg import Int16

from crtp_driver.high_level_commander import HighLevelCommander
from crtp_driver.basic_commander import BasicCommander
from crtp_driver.generic_commander import GenericCommander
from crtp_driver.hardware_commander import HardwareCommander
from crtp_driver.param_reader import ParameterCommander
from crtp_driver.LoggingCommander import LoggingCommander

from crtp_driver.localization import Localization

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from object_tracker_interfaces.srv import AddTrackerObject, RemoveTrackerObject

class Link:
    def __init__(self, send_no_resonse, send_packet_sync, batch_request):
        self.send_no_response = send_no_resonse
        self.send_packet_sync = send_packet_sync
        self.batch_request = batch_request
    
    def send_packet_no_response(self, packet):
        self.send_no_response(packet)
    

    def send_packet(self, packet, expects_response, matching_bytes):
        return self.send_packet_sync(packet, expects_response, matching_bytes)
    
    def send_batch_request(self, packets):
        return self.batch_request(packets)



class Crazyflie(Node):
    STATE_INIT = 0
    STATE_RUNNING = 1
    STATE_DESTROY = 2

    def __init__(self):
        super().__init__("cf")
        self.declare_parameter('id', 0x0)
        self.declare_parameter('channel', 100)
        self.declare_parameter('datarate', 2)
        self.id = self.get_parameter('id').get_parameter_value().integer_value
        self.channel = self.get_parameter('channel').get_parameter_value().integer_value
        self.datarate = self.get_parameter('datarate').get_parameter_value().integer_value
        self.address = (0xE7, 0xE7,0xE7,0xE7, self.id)
        if self.id == 0xE7: self.address = (0xFF, 0xE7,0xE7,0xE7, 0xE7) # TODO very hacky way to test broadcasting
        self.prefix = "cf" + str(self.id)

        self.state = self.STATE_INIT

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history= QoSHistoryPolicy.KEEP_ALL,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.send_packet_service = self.create_client(CrtpPacketSend, "/crazyradio/send_crtp_packet", qos_profile=qos_profile)
        while not self.send_packet_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Send CRTP Packet Service not available, waiting again...")

        # Establish Connection
        for i in range(10):
            self.send_null_packet()

        # Establish Tracking
        self.add_to_tracker_service = self.create_client(AddTrackerObject, "/tracker/add_object")
        self.remove_from_tracker_service = self.create_client(RemoveTrackerObject, "/tracker/remove_object")
        while not self.add_to_tracker_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Add to Tracker Service not available, waiting again...")
        
        req = AddTrackerObject.Request()
        req.tf_name.data = self.prefix
        req.marker_configuration_idx = 4
        req.max_initial_deviation = 0.4
        self.add_to_tracker_service.call_async(req)

        
        link = Link(self.send_crtp_packet_async,self.send_crtp_packet_sync, self.send_crtp_batch_request_sync)

        ## Add necesities in order to initialise
        self.param_reader = ParameterCommander(self, link)
        self.logging_commander = LoggingCommander(self, link)
        self.hardware_commander = HardwareCommander(self, link)
        self.localization = Localization(self, link)      
        self.initialize()

        self.hl_commander = HighLevelCommander(self, link)
        self.basic_commander = BasicCommander(self, link)
        self.generic_commander = GenericCommander(self, link)
        

        self.create_subscription(Int16, "~/set_color", self.set_color,  10)
        self.create_subscription(Int16, "~/send_nullpacket", self.send_null_packet, 10)

        
        second_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Int16, "~/initialize", self.initialize, 10,callback_group=second_cb_group)
        self.create_subscription(Int16, "~/set_parameter", self.initialize, 10,callback_group=second_cb_group)
        

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.2, self.on_timer, callback_group=second_cb_group)

        self.state = self.STATE_RUNNING
        self.get_logger().info("Initialization complete!")

    def __del__(self):
        self.destroy_node()
    
    def _prepare_send_request(self):
        req = CrtpPacketSend.Request()
        req.channel = self.channel
        req.address = self.address
        req.datarate = self.datarate
        return req

    def send_crtp_packet_async(self, packet, expects_response=False, matching_bytes=0):
        req = self._prepare_send_request()
        req.expects_response = expects_response
        req.matching_bytes = matching_bytes        
        req.packet = packet
        return self.send_packet_service.call_async(req)

    def send_crtp_packet_sync(self, packet, expects_response, matching_bytes):
        fut = self.send_crtp_packet_async(packet, expects_response, matching_bytes)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result().packet
    
    def send_crtp_batch_request_sync(self, packets):
        futures = []
        for pkt in packets:
            packet, expects_response, matching_bytes = pkt
            futures.append(self.send_crtp_packet_async(packet, expects_response, matching_bytes))
        
        responses = []
        for fut in futures:
            rclpy.spin_until_future_complete(self, fut)
            responses.append(fut.result())

        return responses
        
    def on_timer(self):
        if self.state == self.STATE_INIT: return
        if self.id == 0xE7: return
        try:
            t = self.tf_buffer.lookup_transform(
                "world",
                self.prefix,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info("Tracker no Frame")
            return

        pos =  [t.transform.translation.x   * 1
               , t.transform.translation.y  * 1
               , t.transform.translation.z  * 1]
        self.localization.send_extpos(pos)
        
        
    
    def initialize(self, msg=None):
        self.get_logger().info("Initializing:")
        # Read Loc CRC, Load param toc, set param 
        for i in range(10):
            self.send_null_packet("-") # TODO remove... but its good to check if all is fine by sending some packets
        
        self.logging_commander.load_toc()
        self.param_reader.get_loc_or_load()

        self.get_logger().info("Setting Parameters:")

        self.param_reader.set_parameter("ring", "effect", 2)
        self.param_reader.set_parameter("commander", "enHighLevel", 1)
        self.param_reader.set_parameter("stabilizer", "estimator", 2) #kalman
        self.param_reader.set_parameter("stabilizer", "controller",2) #1: pid 2: mellinger
        self.param_reader.set_parameter("locSrv", "extPosStdDev", 1e-2) #1e-3 # this allows us to fly with 5 hz only
        self.param_reader.set_parameter("locSrv", "extQuatStdDev", 0.5e-1)
        
        
        self.param_reader.set_parameter("ctrlMel", "kp_xy", 0.4)
        self.param_reader.set_parameter("ctrlMel", "kd_xy", 0.2)
        self.param_reader.set_parameter("ctrlMel", "ki_xy", 0.05)
        self.param_reader.set_parameter("ctrlMel", "i_range_xy", 2.0)
        self.param_reader.set_parameter("ctrlMel", "kR_xy", 70000)
        self.param_reader.set_parameter("ctrlMel", "kw_xy", 20000)
        self.param_reader.set_parameter("ctrlMel", "kR_z", 60000)
        self.param_reader.set_parameter("ctrlMel", "kw_z", 12000)
        self.param_reader.set_parameter("ctrlMel", "ki_m_z", 500)
        self.param_reader.set_parameter("ctrlMel", "i_range_m_z", 1500)
        self.param_reader.set_parameter("ctrlMel", "kd_omega_rp", 200)
        self.param_reader.set_parameter("ctrlMel", "kp_z", 1.25)
        self.param_reader.set_parameter("ctrlMel", "kd_z", 0.4)
        self.param_reader.set_parameter("ctrlMel", "ki_z", 0.05)
        self.param_reader.set_parameter("ctrlMel", "i_range_z", 0.4)
        self.param_reader.set_parameter("ctrlMel", "mass", 0.037)
        self.param_reader.set_parameter("ctrlMel", "massThrust", 132000)
        
        self.get_logger().info("Adding Log Blocks")

        self.logging_commander.add_block(1, None)
        self.logging_commander.start_block(1, 100)
        
        self.param_reader.set_parameter("kalman", "resetEstimation", 1)
        pass


    def send_null_packet(self, msg=None):
        req = self._prepare_send_request()
        req.packet.port = 15
        req.packet.channel = 3 # 0xff
        self.send_packet_service.call_async(req)    


    # Legacy function remove, when time is right
    def set_color(self, msg):
        color = msg.data
        pk = CrtpPacketSend.Request()
        pk.channel = self.channel
        pk.address = self.address
        pk.datarate = self.datarate
        pk.packet.port = 2
        pk.packet.channel = 2
        pk.packet.data[0] = 23 ####0x16 in old
        pk.packet.data[1] = 0x00
        pk.packet.data[2] = color
        pk.packet.data_length = 3
        self.send_packet_service.call_async(pk)

def main():
    rclpy.init()
    cf = Crazyflie()
    while rclpy.ok():
        rclpy.spin_once(cf)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
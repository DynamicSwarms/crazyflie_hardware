#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


import logging

from broadcaster_interface.srv import PosiPoseBroadcastObject

from crtp_driver.high_level_commander import HighLevelCommander
from crtp_driver.basic_commander import BasicCommander
from crtp_driver.generic_commander import GenericCommander
from crtp_driver.link_layer import LinkLayer
from crtp_driver.parameters import Parameters
from crtp_driver.logging import Logging
from crtp_driver.console import Console
from crtp_driver.localization import Localization

from object_tracker_interfaces.srv import AddTrackerObject, RemoveTrackerObject
from .crtp_link_ros import CrtpLinkRos
from rclpy.parameter import Parameter

class Crazyflie(Node):
    STATE_INIT = 0
    STATE_RUNNING = 1
    STATE_DESTROY = 2

    def __init__(self):
        super().__init__("cf", automatically_declare_parameters_from_overrides=True)
        self.id = self.get_parameter('id').get_parameter_value().integer_value
        self.channel = self.get_parameter('channel').get_parameter_value().integer_value
        self.datarate = self.get_parameter('datarate').get_parameter_value().integer_value
        self.initial_position = self.get_parameter('initial_position').get_parameter_value().double_array_value
        self.address = (0xE7, 0xE7,0xE7,0xE7, self.id)
        self.prefix = "cf" + str(self.id)

        self.state = self.STATE_INIT
        self.crtp_link = CrtpLinkRos(self, self.channel, self.address, self.datarate, self.on_link_shutdown)

        self.hardware_commander = LinkLayer(self, self.crtp_link)
        self.console = Console(self, self.crtp_link)

        # Establish Connection
        # We need to send highest priority packets because some packages might get lost in the beginning
        for _ in range(10):
            self.console.send_consolepacket()

        self.add_to_tracker()

        self.add_to_broadcaster()
        
        ## Add Parameters, Logging and Localization which initialize automatically
        self.parameters = Parameters(self, self.crtp_link)
        self.logging = Logging(self, self.crtp_link)
        self.localization = Localization(self, self.crtp_link)
        self.initialize() # Intialize from default parameters

        # Add functionalities after initialization, ensuring we cannot takeoff before initialization
        self.hl_commander = HighLevelCommander(self, self.crtp_link)
        self.basic_commander = BasicCommander(self, self.crtp_link)
        self.generic_commander = GenericCommander(self, self.crtp_link)
        
        self.state = self.STATE_RUNNING
        self.get_logger().info("Initialization complete!")

    def __del__(self):
        self.destroy_node()

    def on_link_shutdown(self):
        self.get_logger().info("Callback for Link Shutdown!")
        self.destroy_node()
        # TODO: Check if this results in any issues in future
    
    def add_to_tracker(self):
        # Establish Tracking
        self.add_to_tracker_service = self.create_client(AddTrackerObject, "/tracker/add_object")
        self.remove_from_tracker_service = self.create_client(RemoveTrackerObject, "/tracker/remove_object")
        while not self.add_to_tracker_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Add to Tracker Service not available, waiting again...")
        
        req = AddTrackerObject.Request()
        req.tf_name.data = self.prefix
        req.marker_configuration_idx = 4
        req.max_initial_deviation = 0.4
        req.initial_pose.position.x, req.initial_pose.position.y, req.initial_pose.position.z = self.initial_position
        self.add_to_tracker_service.call_async(req)


    def add_to_broadcaster(self):
        # Establish Broadcasting
        self.add_to_broadcaster_service = self.create_client(PosiPoseBroadcastObject, "/add_posi_pose_object")
        while not self.add_to_broadcaster_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Add to Broadcaster Service not available, waiting again...")
        
        req = PosiPoseBroadcastObject.Request()
        req.channel = self.channel
        req.tf_frame_id = self.prefix
        req.data_rate = self.datarate
        self.add_to_broadcaster_service.call_async(req)

    def initialize(self, msg=None):
        self.get_logger().info("Initializing:")
        self.get_logger().info("Setting Parameters:")

        default_parameter_names = dict(filter(lambda par: par[0].startswith("default_firmware_params") ,self._parameters.items()))
        for param_name in default_parameter_names:
            param = self.get_parameter(param_name)
            msg = param.to_parameter_msg()
            msg.name = msg.name[len("default_firmware_params."):] 
            self.set_parameters([Parameter.from_parameter_msg(msg)])
            
        self.get_logger().info("Adding Log Blocks")
        self.logging.add_block(1, None)
        self.logging.start_block(1, 100)
        self.parameters.set_parameter("kalman", "resetEstimation", 1)
       
def main():
    rclpy.init()
    cf = Crazyflie()
    while rclpy.ok():
        rclpy.spin_once(cf)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
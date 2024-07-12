#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

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

from crtp_driver.high_level_commander import HighLevelCommander
from crtp_driver.basic_commander import BasicCommander
from crtp_driver.generic_commander import GenericCommander
from crtp_driver.hardware_commander import HardwareCommander
from crtp_driver.param_reader import ParamReader
from crtp_driver.localization import Localization

from crazyflie_interface.msg import SetGroupMask, Takeoff, Land, Stop, GoTo, StartTrajectory, UploadTrajectory # HL_Commander
from crazyflie_interface.msg import NotifySetpointsStop, VelocityWorld, Hover, FullState, Position # (Generic)Commander


import math

from geometry_msgs.msg import Twist


from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class Crazyflie(Node):
    COMMAND_TAKEOFF = 7
    COMMAND_LAND = 8
    SETPOINT_HL = 0x08
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
        self.cf_prefix = 'cf' + str(self.id)


        self.hl_commander = HighLevelCommander()
        self.basic_commander = BasicCommander()
        self.generic_commander = GenericCommander()
        self.hardware_commander = HardwareCommander()

        self.param_reader = ParamReader(self)
        self.localization = Localization()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history= QoSHistoryPolicy.KEEP_ALL,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.send_packet_service = self.create_client(CrtpPacketSend, "crazyradio/send_crtp_packet", qos_profile=qos_profile)
        while not self.send_packet_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Send CRTP Packet Service not available, waiting again...")

        self.create_subscription(Int16, self.cf_prefix + "/set_color", self.set_color,  10)

        self.create_subscription(Takeoff, self.cf_prefix + "/takeoff", self.takeoff, 10)
        self.create_subscription(Land, self.cf_prefix + "/land", self.land, 10)
        self.create_subscription(GoTo, self.cf_prefix + "/go_to", self.go_to, 10)
        self.create_subscription(SetGroupMask, self.cf_prefix + "/set_group_mask", self.set_group_mask, 10)

        self.create_subscription(NotifySetpointsStop, self.cf_prefix + "/notify_setpoints_stop", self.notify_setpoints_stop, 10) 
        self.create_subscription(VelocityWorld, self.cf_prefix + "/cmd_vel", self.cmd_vel, 10)
        self.create_subscription(Hover, self.cf_prefix + "/cmd_hover", self.cmd_hover, 10)
        self.create_subscription(FullState, self.cf_prefix + "/cmd_full_state", self.cmd_full_state, 10)
        self.create_subscription(Position, self.cf_prefix + "/cmd_position", self.cmd_position, 10)

        self.create_subscription(Int16, self.cf_prefix + "/platform_power_down", self.platform_power_down,  10)
        self.create_subscription(Int16, self.cf_prefix + "/reboot_to_fw", self.reboot_to_fw,  10)

        self.create_subscription(Int16, self.cf_prefix + "/send_nullpacket", self.send_null_packet, 10)

        second_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Int16, self.cf_prefix + "/get_loc_toc", self.get_loc_toc, 10, callback_group=second_cb_group)


        self.create_subscription(Int16, self.cf_prefix + "/initialize", self.initialize, 10,callback_group=second_cb_group)
        self.create_subscription(Int16, self.cf_prefix + "/set_parameter", self.initialize, 10,callback_group=second_cb_group)
        

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.on_timer, callback_group=second_cb_group)
    def __del__(self):
        self.destroy_node()
    
    def on_timer(self):
        if self.id == 0xE7: return
        try:
            t = self.tf_buffer.lookup_transform(
                "cf",
                "world",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info("Tracker no Frame")
            return
        pos =  [t.transform.translation.x   * 1
               , t.transform.translation.y  * 1
               , t.transform.translation.z  * 1]

        req = self._prepare_send_request()
        req.packet = self.localization.send_extpos(pos)
        self.send_packet_service.call_async(req)



        
    def _prepare_send_request(self):
        req = CrtpPacketSend.Request()
        req.channel = self.channel
        req.address = self.address
        req.datarate = self.datarate
        return req
    def initialize(self, msg):
        # Read Loc CRC, Load param toc, set param 
        for i in range(10):
            self.send_null_packet("-") # TODO remove... but its good to check if all is fine by sending some packets
        self.param_reader.get_loc_or_load()
        self.param_reader.set_parameter("ring", "effect", msg.data)
        self.param_reader.set_parameter("commander", "enHighLevel", 1)
        self.param_reader.set_parameter("stabilizer", "estimator", 2) #kalman
        self.param_reader.set_parameter("stabilizer", "controller",2) #1: pid 2: mellinger
        self.param_reader.set_parameter("locSrv", "extPosStdDev", 1e-3)
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
        
        
        
        self.param_reader.set_parameter("kalman", "resetEstimation", 1)
        pass


    def send_null_packet(self, msg):
        req = self._prepare_send_request()
        req.packet.port = 15
        req.packet.channel = 3 # 0xff
        self.send_packet_service.call_async(req)

    def get_loc_toc(self, msg):
        self.param_reader.get_loc_toc()
  
    def takeoff(self, msg):
        req = self._prepare_send_request()
        duration = msg.duration.sec + msg.duration.nanosec * 1e-9
        req.packet = self.hl_commander.takeoff(msg.height, duration, msg.group_mask, msg.yaw) 
        self.get_logger().info(str(req.packet))

        self.send_packet_service.call_async(req)

    
    def land(self, msg):
        req = self._prepare_send_request()
        duration = msg.duration.sec + msg.duration.nanosec * 1e-9
        req.packet = self.hl_commander.land(msg.height, duration, msg.group_mask, msg.yaw)
        self.send_packet_service.call_async(req)

    def go_to(self, msg):
        req = self._prepare_send_request()
        duration = msg.duration.sec + msg.duration.nanosec * 1e-9
        req.packet = self.hl_commander.go_to(msg.goal.x, msg.goal.y, msg.goal.z, msg.yaw, duration, msg.relative, msg.group_mask)
        self.send_packet_service.call_async(req)

    def set_group_mask(self, msg):
        req = self._prepare_send_request()
        req.packet = self.hl_commander.set_group_mask(msg.group_mask)
        self.send_packet_service.call_async(req)
    
    def notify_setpoints_stop(self, msg):
        req = self._prepare_send_request()
        #TODO: check if group mask might be used
        req.packet = self.generic_commander.send_notify_setpoint_stop(msg.remain_valid_millisecs)
        self.send_packet_service.call_async(req)

    def cmd_vel(self, msg):
        req = self._prepare_send_request()
        req.packet = self.generic_commander.send_velocity_world_setpoint(msg.vel.x, msg.vel.y, msg.vel.z, msg.yaw_rate)
        self.send_packet_service.call_async(req)

    def cmd_hover(self, msg):
        req = self._prepare_send_request()
        req.packet = self.generic_commander.send_hover_setpoint(msg.vx, msg.vy, msg.yawrate, msg.z_distance)
        self.send_packet_service.call_async(req)

    def cmd_full_state(self, msg):
        req = self._prepare_send_request()
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
        acc = [msg.acc.x, msg.acc.y, msg.acc.z]
        orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w ]
        rollRate = msg.twist.angular.x
        pitchRate = msg.twist.angular.y
        yawRate = msg.twist.angular.z
        req.packet = self.generic_commander.send_full_state_setpoint(pos, vel, acc, orientation, rollRate, pitchRate, yawRate)
        self.send_packet_service.call_async(req)


    def cmd_position(self, msg):
        req = self._prepare_send_request()
        req.packet = self.generic_commander.send_position_setpoint(msg.x, msg.y, msg.z, msg.yaw)
        self.send_packet_service.call_async(req)


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

    def platform_power_down(self, msg):
        req = self._prepare_send_request()
        req.packet = self.hardware_commander.platform_power_down()
        self.send_packet_service.call_async(req)

    def reboot_to_fw(self, msg):
        req = self._prepare_send_request()
        req.packet = self.hardware_commander.reset_init()
        self.send_packet_service.call_async(req)
        req.packet = self.hardware_commander.reset_to_bootloader()
        self.send_packet_service.call_async(req)



def main():
    rclpy.init()
    cf = Crazyflie()
    while rclpy.ok():
        rclpy.spin_once(cf)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
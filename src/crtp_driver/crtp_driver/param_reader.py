
import struct 
import rclpy

from crtp_driver.crtp_packer import CrtpPacker
from crtplib.logic.parameters_logic import ParametersLogic
from std_msgs.msg import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


import os

class ParameterCommander(ParametersLogic):
    def __init__(self, node, CrtpLink):
        p = os.environ.get("HOME")
        path = os.path.join(p, ".crazyflie", "param")
        super().__init__(CrtpPacker, CrtpLink, path)
        self.node = node

        callback_group = MutuallyExclusiveCallbackGroup()
        node.create_subscription(Empty, "~/get_parameters_toc_info", self._get_toc_info, 10, callback_group=callback_group)
        node.create_subscription(Empty, "~/download_parameters_toc", self._download_toc, 10, callback_group=callback_group)

    def _download_toc(self, msg):
         self.download_toc_items()

    def _get_toc_info(self, msg):
        nbr_of_items, crc = self.get_toc_info()
        self.node.get_logger().info(str("NBR of Items: "+ str(nbr_of_items)))

    def initialize_toc(self):
        super().initialize_toc()
        for group in self.toc.toc:
                for name in self.toc.toc[group]:
                    self.node.declare_parameter(str(group) + "." + str(name), rclpy.Parameter.Type.DOUBLE)     
    
  
    


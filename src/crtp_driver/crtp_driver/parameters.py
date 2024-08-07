
import struct 
import rclpy

from crtp_driver.crtp_packer import CrtpPacker
from crtplib.logic.parameters_logic import ParametersLogic
from std_msgs.msg import Empty, Int16
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import SetParametersResult

import os

class Parameters(ParametersLogic):
    def __init__(self, node, CrtpLink):
        p = os.environ.get("HOME")
        path = os.path.join(p, ".crazyflie", "param")
        super().__init__(CrtpPacker, CrtpLink, path)
        self.node = node

        self.initialize_toc()

        callback_group = MutuallyExclusiveCallbackGroup()
        node.create_subscription(Empty, "~/get_parameters_toc_info", self._get_toc_info, 10, callback_group=callback_group)
        node.create_subscription(Empty, "~/download_parameters_toc", self._download_toc, 10, callback_group=callback_group)

        #self.create_subscription(Int16, "~/set_parameter", self.initialize, 10,callback_group=second_cb_group)

        node.create_subscription(Int16, "~/set_color", self._set_color,  10, callback_group=callback_group)

        node.add_on_set_parameters_callback(self.set_parameter_callback)


    def _download_toc(self, msg):
         self.download_toc_items()

    def _get_toc_info(self, msg):
        nbr_of_items, crc = self.get_toc_info()
        self.node.get_logger().info(str("NBR of Items: "+ str(nbr_of_items)))

    def set_parameter_callback(self, params):
        for param in params:
              group, name = param.name.split(".")
              self.set_parameter(group, name, param.value)
        return SetParametersResult(successful=True)

    def initialize_toc(self):
        super().initialize_toc()
        for group in self.toc.toc:
                for name in self.toc.toc[group]:
                    toc_element = self.toc.get_element(group, name)
                    if toc_element.pytype == '<f':
                        par_type = rclpy.Parameter.Type.DOUBLE     
                    else:
                        par_type = rclpy.Parameter.Type.INTEGER
                    self.node.declare_parameter(str(group) + "." + str(name), par_type)
                       

    # Legacy function remove, when time is right
    def _set_color(self, msg):
        color = msg.data
        self.set_parameter("ring", "effect", color)
  
    


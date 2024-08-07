import os

from crtp_driver.crtp_packer import CrtpPacker

from std_msgs.msg import Int16, Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from crtplib.logic.logging_logic import LoggingLogic

class Logging(LoggingLogic):
    def __init__(self, node, CrtpLink):
        p = os.environ.get("HOME")
        path = os.path.join(p, ".crazyflie", "log")
        super().__init__(CrtpPacker, CrtpLink, path)
        self.node = node

        self.initialize_toc()

        callback_group = MutuallyExclusiveCallbackGroup()

        node.create_subscription(Empty, "~/get_logging_toc_info", self._get_toc_info, 10, callback_group=callback_group)
        node.create_subscription(Int16, "~/download_logging_toc", self._download_toc, 10, callback_group=callback_group)   

    def _download_toc(self, msg):
        self.download_toc_items()
    
    def _get_toc_info(self, msg):
        nbr_of_items, crc = self.get_toc_info()
        self.node.get_logger().info(str("NBR of Items: "+ str(nbr_of_items)))


    
    


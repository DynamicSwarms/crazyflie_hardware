from typing import List
from crazyflie_interfaces_python.server.logblock import LogBlockServer
from rclpy.node import Node
from .crtp_link_ros import CrtpLinkRos
from std_msgs.msg import Int16, Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from .crtp_packer_ros import CrtpPackerRos
from crtp.logic.logging_logic import LoggingLogic
from crazyflie_interfaces_python.server import LoggingServer
import os


class Logging(LoggingServer, LoggingLogic):
    def __init__(self, node: Node, crtp_link: CrtpLinkRos):
        p = os.environ.get("HOME")
        path = os.path.join(p, ".crazyflie", "log")
        LoggingLogic.__init__(self, CrtpPackerRos, crtp_link, path)
        LoggingServer.__init__(self, node)

        self.initialize_toc()

    # Override
    def download_toc(self) -> None:
        self.send_download_toc_items()

    # Override
    def get_toc_info(self) -> None:
        nbr_of_items, crc = self.send_get_toc_info()
        self.node.get_logger().info(str("NBR of Items: " + str(nbr_of_items)))

    # Override
    def create_log_block(self, variables: List[str], log_block: LogBlockServer) -> None:
        return super().create_log_block(variables, log_block)

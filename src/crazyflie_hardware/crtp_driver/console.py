from rclpy.node import Node

from crtp_driver.crtp_link_ros import CrtpLinkRos
from .crtp_packer_ros import CrtpPackerRos
from crtp.logic.console_logic import ConsoleLogic
from crazyflie_interfaces_python.server import ConsoleServer


class Console(ConsoleServer, ConsoleLogic):
    def __init__(self, node: Node, CrtpLink: CrtpLinkRos):
        ConsoleLogic.__init__(self, CrtpPackerRos, CrtpLink)
        ConsoleServer.__init__(self, node)

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import rclpy.time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .crtp_link_ros import CrtpLinkRos
from .crtp_packer_ros import CrtpPackerRos
from crtp.logic.localization_logic import LocalizationLogic


class Localization(LocalizationLogic):
    def __init__(self, node: Node, crtp_link: CrtpLinkRos, prefix: str):
        super().__init__(CrtpPackerRos, crtp_link)
        self.node: Node = node
        self.prefix: str = prefix
        self.callback_group = MutuallyExclusiveCallbackGroup()

        self.state = 0

    def start_selflocalization(self, tf_name):
        # ??? self.localization_mode_self = set

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.timer = self.node.create_timer(
            0.2, self.on_timer, callback_group=self.callback_group
        )

    def on_timer(self):
        if self.state == 1:
            return
        try:
            t = self.tf_buffer.lookup_transform("world", self.prefix, rclpy.time.Time())
        except TransformException as ex:
            self.node.get_logger().info("Tracker no Frame")
            return

        pos = [
            t.transform.translation.x * 1,
            t.transform.translation.y * 1,
            t.transform.translation.z * 1,
        ]
        self.send_extpos(pos)

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

from broadcaster_interfaces.srv import PosiPoseBroadcastObject

from typing import List


class Localization(LocalizationLogic):
    def __init__(self, node: Node, crtp_link: CrtpLinkRos, prefix: str):
        super().__init__(CrtpPackerRos, crtp_link)
        self.node: Node = node
        self.prefix: str = prefix
        self.callback_group = MutuallyExclusiveCallbackGroup()

        self.state = 0

    def start_selflocalization(self, tf_name):
        """It is also possible to not use broadcaster for sending external pose.
        This does this but is not fully implemented
        Args:
            tf_name (_type_): _description_
        """

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

    def start_external_tracking(
        self,
        marker_configuration_index: int,
        dynamics_configuration_index: int,
        max_initial_deviation: float,
        initial_position: List[float],
        channel: int,
        datarate: int,
    ):
        self.add_to_tracker(
            marker_configuration_index,
            dynamics_configuration_index,
            max_initial_deviation,
            initial_position,
        )
        self.add_to_broadcaster(channel, datarate)

    def add_to_tracker(
        self,
        marker_configuration_index: int,
        dynamics_configuration_index: int,
        max_initial_deviation: float,
        initial_position: List[float],
    ):
        from object_tracker_interfaces.srv import AddTrackerObject, RemoveTrackerObject

        # Establish Tracking
        self.add_to_tracker_service = self.node.create_client(
            srv_type=AddTrackerObject,
            srv_name="/tracker/add_object",
            callback_group=self.callback_group,
        )
        self.remove_from_tracker_service = self.node.create_client(
            srv_type=RemoveTrackerObject,
            srv_name="/tracker/remove_object",
            callback_group=self.callback_group,
        )
        while not self.add_to_tracker_service.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                "Add to Tracker Service not available, waiting again..."
            )

        req = AddTrackerObject.Request()
        req.tf_name.data = self.prefix
        req.marker_configuration_idx = marker_configuration_index
        req.dynamics_configuration_idx = dynamics_configuration_index
        req.max_initial_deviation = max_initial_deviation
        (
            req.initial_pose.position.x,
            req.initial_pose.position.y,
            req.initial_pose.position.z,
        ) = initial_position
        self.add_to_tracker_service.call_async(req)

    def add_to_broadcaster(self, channel: int, datarate: int):
        # Establish Broadcasting
        self.add_to_broadcaster_service = self.node.create_client(
            srv_type=PosiPoseBroadcastObject,
            srv_name="/add_posi_pose_object",
            callback_group=self.callback_group,
        )
        while not self.add_to_broadcaster_service.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                "Add to Broadcaster Service not available, waiting again..."
            )

        req = PosiPoseBroadcastObject.Request()
        req.channel = channel
        req.data_rate = datarate
        req.tf_frame_id = self.prefix
        self.add_to_broadcaster_service.call_async(req)

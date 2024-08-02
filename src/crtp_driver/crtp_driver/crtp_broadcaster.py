#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from broadcaster_interface.srv import PosiPoseBroadcastObject
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Broadcaster(Node):
    def __init__(self):
        super().__init__("broadcaster")
        self.world = self.declare_parameter('world', 'world').get_parameter_value().string_value
        self.position_only = True  # self.declare_parameter('position_only', True).get_parameter_value().bool_value
        self.hz = self.declare_parameter('hz', 10).get_parameter_value().integer_value

        self.add_service = self.create_service(PosiPoseBroadcastObject, "add_posi_pose_object", self.add_object)
        self.remove_service = self.create_service(PosiPoseBroadcastObject, "remove_posi_pose_object", self.remove_object)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0 / self.hz, self.run)
        self.channels = set()
        self.frames = set()

    def add_object(self, request, response):
        if request.channel != 0:
            self.channels.add(request.channel)
        self.frames.add(request.tf_frame_id)
        response.success = True
        return response

    def remove_object(self, request, response):
        self.frames.remove(request.tf_frame_id)
        response.success = True
        return response

    def run(self):
        rclpy.logging.get_logger("broadcaster").info("Running")
        for pos in self._get_external_positions():
            string = " ".join([str(p) for p in pos])
            rclpy.logging.get_logger("broadcaster").info(string)
        pass

    def _get_external_positions(self):
        for frame in self.frames:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.world,
                    frame,
                    rclpy.time.Time())
            except TransformException as ex:
                continue
            pos = [t.transform.translation.x,
                   t.transform.translation.y,
                   t.transform.translation.z]
            yield pos


def main(args=None):
    rclpy.init(args=args)
    node = Broadcaster()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
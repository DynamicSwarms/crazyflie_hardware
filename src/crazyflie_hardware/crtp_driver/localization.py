import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


from .crtp_packer_ros import CrtpPackerRos
from crtp.logic.localization_logic import LocalizationLogic

class Localization(LocalizationLogic):
    def __init__(self, node, CrtpLink):
        super().__init__(CrtpPackerRos, CrtpLink)
        self.node = node
        self.callback_group = MutuallyExclusiveCallbackGroup()
        
    def start_selflocalization(self, tf_name):
        self.localization_mode_self = set

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.2, self.on_timer, callback_group=self.callback_group)

    
    def on_timer(self):
        if self.state == self.STATE_INIT: return
        if self.id == 0xE7: return
        try:
            t = self.tf_buffer.lookup_transform(
                "world",
                self.prefix,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info("Tracker no Frame")
            return

        pos =  [t.transform.translation.x   * 1
               , t.transform.translation.y  * 1
               , t.transform.translation.z  * 1]
        self.send_extpos(pos)
        





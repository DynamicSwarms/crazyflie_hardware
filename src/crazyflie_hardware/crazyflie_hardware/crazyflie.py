#!/usr/bin/env python3
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import State as LifecycleState

from broadcaster_interfaces.srv import PosiPoseBroadcastObject

from crtp_driver.crtp_link_ros import CrtpLinkRos

from crtp_driver.high_level_commander import HighLevelCommander
from crtp_driver.basic_commander import BasicCommander
from crtp_driver.generic_commander import GenericCommander
from crtp_driver.link_layer import LinkLayer
from crtp_driver.parameters import Parameters
from crtp_driver.logging import Logging
from crtp_driver.console import Console
from crtp_driver.localization import Localization

from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

from typing import List, Any

from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor


class Crazyflie(LifecycleNode):
    STATE_INIT = 0
    STATE_RUNNING = 1
    STATE_DESTROY = 2

    def __init__(self, executor: SingleThreadedExecutor):
        """Initializes a crazyflie

        Args:
            executor (SingleThreadedExecutor): The executor used to spin this node, we'll shutdown the executor in order to stop us.
        """
        super().__init__("cf")
        self.__declare_parameters()
        self.get_logger().fatal(f"Crazyflie Launching!: {self.id}, {self.channel}")

        self.address = (0xE7, 0xE7, 0xE7, 0xE7, self.id)
        self.prefix = "cf" + str(self.id)

        self.state = self.STATE_INIT
        self.executor = executor

        state_transition = self.create_client(
            ChangeState,
            f"{self.get_name()}/change_state",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        request = ChangeState.Request()
        request.transition.id = LifecycleState.TRANSITION_STATE_CONFIGURING
        request.transition.label = "configure"
        state_transition.call_async(request)

    def configure(self):
        self.crtp_link = CrtpLinkRos(
            self, self.channel, self.address, self.datarate, self.on_link_shutdown
        )
        self.hardware_commander = LinkLayer(self, self.crtp_link)
        self.console = Console(self, self.crtp_link)

        # Establish Connection
        # We need to send highest priority packets because some packages might get lost in the beginning
        for _ in range(10):
            self.console.send_consolepacket()

        if self.send_external_position:
            self.initialize_tracking()

        ## Add Parameters, Logging and Localization which initialize automatically
        self.parameters = Parameters(self, self.crtp_link)
        self.logging = Logging(self, self.crtp_link)
        self.crtp_link.add_callback(5, self.logging.crtp_callback)
        self.localization = Localization(self, self.crtp_link, self.prefix)
        self.initialize()  # Intialize from default parameters

        # Add functionalities after initialization, ensuring we cannot takeoff before initialization
        self.hl_commander = HighLevelCommander(self, self.crtp_link)
        self.basic_commander = BasicCommander(self, self.crtp_link)
        self.generic_commander = GenericCommander(self, self.crtp_link)

        self.state = self.STATE_RUNNING
        self.get_logger().info("Initialization complete!")

    def on_link_shutdown(self):
        self.executor.shutdown(timeout_sec=0.1)

    def initialize(self, msg=None):
        self.get_logger().info("Initializing:")
        self.get_logger().info("Setting Parameters:")

        default_parameter_names = dict(
            filter(
                lambda par: par[0].startswith("default_firmware_params"),
                self._parameters.items(),
            )
        )
        for param_name in default_parameter_names:
            param = self.get_parameter(param_name)
            msg = param.to_parameter_msg()
            msg.name = msg.name[len("default_firmware_params.") :]
            self.set_parameters([Parameter.from_parameter_msg(msg)])

        self.get_logger().info("Adding Log Blocks")
        self.parameters.set_parameter("kalman", "resetEstimation", 1)

    def initialize_tracking(self):
        self.add_to_tracker()
        self.add_to_broadcaster()

    def add_to_tracker(self):
        from object_tracker_interfaces.srv import AddTrackerObject, RemoveTrackerObject

        # Establish Tracking
        self.add_to_tracker_service = self.create_client(
            AddTrackerObject, "/tracker/add_object"
        )
        self.remove_from_tracker_service = self.create_client(
            RemoveTrackerObject, "/tracker/remove_object"
        )
        while not self.add_to_tracker_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Add to Tracker Service not available, waiting again..."
            )

        req = AddTrackerObject.Request()
        req.tf_name.data = self.prefix
        req.marker_configuration_idx = self.marker_configuration_index
        req.dynamics_configuration_idx = self.dynamics_configuration_index
        req.max_initial_deviation = self.max_initial_deviation
        (
            req.initial_pose.position.x,
            req.initial_pose.position.y,
            req.initial_pose.position.z,
        ) = self.initial_position
        self.add_to_tracker_service.call_async(req)

    def add_to_broadcaster(self):
        # Establish Broadcasting
        self.add_to_broadcaster_service = self.create_client(
            PosiPoseBroadcastObject, "/add_posi_pose_object"
        )
        while not self.add_to_broadcaster_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Add to Broadcaster Service not available, waiting again..."
            )

        req = PosiPoseBroadcastObject.Request()
        req.channel = self.channel
        req.tf_frame_id = self.prefix
        req.data_rate = self.datarate
        self.add_to_broadcaster_service.call_async(req)

    def __declare_parameters(self):
        self.__declare_readonly_parameter("id", 0xE7)
        self.__declare_readonly_parameter("channel", 80)
        self.__declare_readonly_parameter("initial_position", [0.0, 0.0, 0.0])
        self.__declare_readonly_parameter("datarate", 2)

        # Parameters from crazyflie types.yaml
        self.__declare_readonly_parameter("send_external_position", False)
        self.__declare_readonly_parameter("send_external_pose", False)
        self.__declare_readonly_parameter("max_initial_deviation", 1.0)
        self.__declare_readonly_parameter("marker_configuration_index", 4)
        self.__declare_readonly_parameter("dynamics_configuration_index", 0)

        # Parameters from yaml
        # We need to explicitly iterate over the list of firmware parameters and add them
        for firmware_param_name, firmware_param_value in filter(
            lambda item: item[0].startswith("default_firmware_params"),
            self._parameter_overrides.items(),
        ):
            self.declare_parameter(
                name=firmware_param_name,
                value=firmware_param_value,
                descriptor=ParameterDescriptor(dynamic_typing=True, read_only=True),
            )

    def __declare_readonly_parameter(self, name: str, value: Any):
        self.declare_parameter(
            name=name, value=value, descriptor=ParameterDescriptor(read_only=True)
        )

    @property
    def id(self) -> int:
        return self.get_parameter("id").get_parameter_value().integer_value

    @property
    def channel(self) -> int:
        return self.get_parameter("channel").get_parameter_value().integer_value

    @property
    def datarate(self) -> int:
        return self.get_parameter("datarate").get_parameter_value().integer_value

    @property
    def initial_position(self) -> List[float]:
        return (
            self.get_parameter("initial_position")
            .get_parameter_value()
            .double_array_value
        )

    @property
    def send_external_position(self) -> bool:
        return (
            self.get_parameter("send_external_position")
            .get_parameter_value()
            .bool_value
        )

    @property
    def send_external_pose(self) -> bool:
        return self.get_parameter("send_external_pose").get_parameter_value().bool_value

    @property
    def max_initial_deviation(self) -> float:
        return (
            self.get_parameter("max_initial_deviation")
            .get_parameter_value()
            .double_value
        )

    @property
    def marker_configuration_index(self) -> int:
        return (
            self.get_parameter("marker_configuration_index")
            .get_parameter_value()
            .integer_value
        )

    @property
    def dynamics_configuration_index(self) -> int:
        return (
            self.get_parameter("dynamics_configuration_index")
            .get_parameter_value()
            .integer_value
        )

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(
            f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'configure'"
        )
        self.configure()
        self.get_logger().info("State transition done")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(
            f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'activate'"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(
            f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'deactivate'"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(
            f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'shutdown'"
        )
        self.shutdown = True
        self.cf.close_crazyflie()
        return TransitionCallbackReturn.SUCCESS


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()

    cf = Crazyflie(executor)
    try:
        while rclpy.ok() and not executor._is_shutdown:
            rclpy.spin_once(cf, timeout_sec=0.1, executor=executor)
        rclpy.try_shutdown()
    except KeyboardInterrupt:
        pass  # Do not print Message on shutdown because we get closed on demand
    exit()


if __name__ == "__main__":
    main()

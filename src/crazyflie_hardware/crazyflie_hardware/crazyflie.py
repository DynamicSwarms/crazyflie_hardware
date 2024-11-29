#!/usr/bin/env python3
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import State as LifecycleState
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

from crtp_driver.high_level_commander import HighLevelCommander
from crtp_driver.basic_commander import BasicCommander
from crtp_driver.generic_commander import GenericCommander
from crtp_driver.link_layer import LinkLayer
from crtp_driver.parameters import Parameters
from crtp_driver.logging import Logging
from crtp_driver.console import Console
from crtp_driver.localization import Localization

from crtp_driver.crtp_link_ros import CrtpLinkRos

from typing import List, Any


class Crazyflie(LifecycleNode):

    def __init__(self, executor: SingleThreadedExecutor):
        """Initializes a crazyflie

        Args:
            executor (SingleThreadedExecutor): The executor used to spin this node, we'll shutdown the executor in order to stop us.
        """
        super().__init__("cf")
        self.executor = executor
        self.__declare_parameters()

        self.get_logger().info(f"Crazyflie Launching!: {self.id}, {self.channel}")
        self.prefix = "cf" + str(self.id)

        self.__trigger_state_transition(
            LifecycleState.TRANSITION_STATE_CONFIGURING, "configure"
        )
        self.__trigger_state_transition(
            LifecycleState.TRANSITION_STATE_ACTIVATING, "activate"
        )

    def configure(self):
        self.crtp_link = CrtpLinkRos(
            self,
            self.channel,
            (0xE7, 0xE7, 0xE7, 0xE7, self.id),
            self.datarate,
            self.on_link_shutdown,
        )
        self.hardware_commander = LinkLayer(self, self.crtp_link)
        self.console = Console(self, self.crtp_link)

        # Establish Connection
        # We need to send highest priority packets because some packages might get lost in the beginning
        for _ in range(10):
            self.console.send_consolepacket()

        ## Add Parameters, Logging and Localization which initialize automatically
        self.parameters = Parameters(self, self.crtp_link)
        self.logging = Logging(self, self.crtp_link)
        self.localization = Localization(self, self.crtp_link, self.prefix)

        # Start localization services / broadcasting if wished for
        if self.send_external_position:
            self.localization.start_external_tracking(
                self.marker_configuration_index,
                self.dynamics_configuration_index,
                self.max_initial_deviation,
                self.initial_position,
                self.channel,
                self.datarate,
            )

        self.set_default_parameters()
        # Add functionalities after initialization, ensuring we cannot takeoff before initialization
        self.hl_commander = HighLevelCommander(self, self.crtp_link)
        self.basic_commander = BasicCommander(self, self.crtp_link)
        self.generic_commander = GenericCommander(self, self.crtp_link)

        self.get_logger().info("Configuring complete!")

    def on_link_shutdown(self):
        self.executor.shutdown(timeout_sec=0.1)

    def set_default_parameters(self, msg=None):
        self.get_logger().info("Setting default frimware parameters.")

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

        self.parameters.set_parameter("kalman", "resetEstimation", 1)

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

    # Lifecycle Overrides
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Crazyflie {self.id} configuring.")
        self.configure()
        # The configuration cannot fail directly. There will however be a "on link shutdown" called.
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Crazyflie {self.id} transitioned to active.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Crazyflie {self.id} deactivating (not implemented)")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Crazyflie {self.id} shutting down.")
        self.executor.shutdown(timeout_sec=0.1)
        return TransitionCallbackReturn.SUCCESS

    def __trigger_state_transition(self, state: LifecycleState, label: str):
        state_transition = self.create_client(
            ChangeState,
            f"{self.get_name()}/change_state",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        request = ChangeState.Request()
        request.transition.id = state
        request.transition.label = label
        state_transition.call_async(request)


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

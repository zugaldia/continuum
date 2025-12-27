import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.publisher import Publisher
from sensor_msgs.msg import Joy
from std_msgs.msg import Header

from continuum.constants import (
    QOS_DEPTH_DEFAULT,
    TOPIC_JOYSTICK_BUTTON_EVENT,
    TOPIC_JOYSTICK_AXIS_EVENT,
    CONTINUUM_NAMESPACE,
)
from continuum.models import JoystickButton, JoystickAxisDirection, JoystickAxis, JoystickAxisState, JoystickButtonState
from continuum_core.shared.base_node import BaseNode
from continuum_interfaces.msg import JoystickButtonEvent, JoystickAxisEvent

JOY_BUTTON_MAPPING = {
    "DEFAULT": {  # 8BitDo SN30 Pro
        0: JoystickButton.BUTTON_B,
        1: JoystickButton.BUTTON_A,
        2: JoystickButton.BUTTON_Y,
        3: JoystickButton.BUTTON_X,
        4: JoystickButton.BUTTON_L,
        5: JoystickButton.BUTTON_R,
        6: JoystickButton.BUTTON_SELECT,
        7: JoystickButton.BUTTON_START,
        8: JoystickButton.BUTTON_HOME,
    },
}

JOY_AXIS_MAPPING = {
    "DEFAULT": {  # 8BitDo SN30 Pro
        6: JoystickAxisDirection.LEFT_RIGHT,
        7: JoystickAxisDirection.UP_DOWN,
    },
}

# Threshold for axis activation (absolute value must exceed this to trigger event)
AXIS_ACTIVATION_THRESHOLD = 0.5


class JoystickNode(BaseNode):
    _button_publisher: Publisher[JoystickButtonEvent]
    _axis_publisher: Publisher[JoystickAxisEvent]

    def __init__(self):
        super().__init__("joystick_node")
        self.set_node_info(name="Joystick Node", description="Publish joystick events for both buttons and axes")

        # We need to make this configurable
        controller_name = "DEFAULT"
        self._button_mapping = JOY_BUTTON_MAPPING[controller_name]
        self._axis_mapping = JOY_AXIS_MAPPING[controller_name]

        self.get_logger().info("Joystick node initialized.")

    def register_publishers(self) -> None:
        """Register the joystick event publishers."""
        self._button_publisher = self.create_publisher(
            JoystickButtonEvent, TOPIC_JOYSTICK_BUTTON_EVENT, QOS_DEPTH_DEFAULT
        )
        self._axis_publisher = self.create_publisher(JoystickAxisEvent, TOPIC_JOYSTICK_AXIS_EVENT, QOS_DEPTH_DEFAULT)

    def register_subscribers(self) -> None:
        """Register the subscriber for Joy events."""
        self.create_subscription(Joy, f"/{CONTINUUM_NAMESPACE}/joy", self._joy_callback, QOS_DEPTH_DEFAULT)

    def on_shutdown(self) -> None:
        """Clean up joystick node resources."""
        self.get_logger().info("Joystick node shutting down.")

    def _joy_callback(self, msg: Joy):
        # Buttons
        for index, value in enumerate(msg.buttons):
            if value == 1:  # Button is pressed (value == 0 when released)
                if index in self._button_mapping:
                    button = self._button_mapping[index]
                    self._handle_button(msg.header, button)
                else:
                    self.get_logger().warning(f"{index} clicked: Unrecognized.")

        # Axes
        for index, value in enumerate(msg.axes):
            if index in self._axis_mapping:
                axis = self._axis_mapping[index]
                if axis == JoystickAxisDirection.LEFT_RIGHT:
                    if value < -AXIS_ACTIVATION_THRESHOLD:
                        self._handle_axis(msg.header, JoystickAxis.AXIS_RIGHT)
                    elif value > AXIS_ACTIVATION_THRESHOLD:
                        self._handle_axis(msg.header, JoystickAxis.AXIS_LEFT)
                elif axis == JoystickAxisDirection.UP_DOWN:
                    if value < -AXIS_ACTIVATION_THRESHOLD:
                        self._handle_axis(msg.header, JoystickAxis.AXIS_DOWN)
                    elif value > AXIS_ACTIVATION_THRESHOLD:
                        self._handle_axis(msg.header, JoystickAxis.AXIS_UP)

    def _handle_button(self, header: Header, button: JoystickButton):
        event = JoystickButtonEvent()
        event.header = header
        event.button = button.value
        event.state = JoystickButtonState.PRESSED.value
        self._button_publisher.publish(event)
        self.get_logger().info(f"{button} clicked.")

    def _handle_axis(self, header: Header, axis: JoystickAxis):
        event = JoystickAxisEvent()
        event.header = header
        event.axis = axis.value
        event.state = JoystickAxisState.PRESSED.value
        self._axis_publisher.publish(event)
        self.get_logger().info(f"{axis} moved.")


def main(args=None):
    try:
        with rclpy.init(args=args):
            joystick_node = JoystickNode()
            rclpy.spin(joystick_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()

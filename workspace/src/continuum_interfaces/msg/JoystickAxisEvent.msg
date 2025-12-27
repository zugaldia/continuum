# Joystick Axis Event Message

std_msgs/Header header  # Standard ROS header with timestamp and frame_id
string axis             # The axis direction (e.g., AXIS_UP, AXIS_DOWN, AXIS_LEFT, AXIS_RIGHT)
string state            # The state of the axis (e.g., PRESSED, RELEASED)

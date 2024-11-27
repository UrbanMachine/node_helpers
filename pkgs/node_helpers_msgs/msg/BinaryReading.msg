# This message is used to publish binary value sensor messages that follow the BaseSensor message structure

std_msgs/Header header

bool value

# A few constants to help readability in code for different sensor use cases:
# For binary presence detection sensors
bool BLOCKED = true
bool UNBLOCKED = false
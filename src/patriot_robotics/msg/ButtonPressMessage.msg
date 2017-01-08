# Which hand to use (see enum info below)
uint8 side

# Total time to spend moving the hand. This does not include time spent 
# pressing the button. If not provided, a reasonable default will be used.
float64 motion_time

# Location of the button in world coordinates
geometry_msgs/Vector3 button_position

# This message utilizes "enums". Enum value information for this message follows.

# "side" enum values:
uint8 LEFT=0 # refers to the LEFT side of a robot
uint8 RIGHT=1 # refers to the RIGHT side of a robot

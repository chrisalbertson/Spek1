"""
This file holds only CONSTANTS

Our definition of "constant" is a value that does not change after execution begins.
so some code to get the constant values is allowed
"""

from enum import Enum

# This defines if the real robot hardware is present.
# If not the controller will not attempt to send data to or
# interact with physical device nor will it import the
# associated library modules.
UsePCA9685Hardware = True

body_number_of_legs = 4

# This the maximum forward speed the robot can walk.  Units are meters per second
max_forward_speed = 0.10

# on the Spot Micro, the second joint axis does NOT intersect the first but
# is vertically offset about 20 to 15 mm.    In many quadruped models the
#  J1 to J2 distance is just "l1" but SpotMicro needs FOUR constants
j1_to_j2_horizontal_offset = 0.060     # l1
j1_to_j2_vertical_offset   = 0.015     # l2
leg_upper_length           = 0.115     # l3
leg_lower_length           = 0.130     # l4


# This defines the "Wiring Diagram", that is which servo chanel is connected to which joint
# on which leg.  First we define some constants using enums
class leg_id (Enum):
    RF = 0
    LF = 1
    RR = 2
    LR = 3

class joint_id (Enum):
    THETA1 = 0
    THETA2 = 1
    THETA3 = 2


# the get the servo chanel for the Right Front leg's knee joint use this:
# channel = servo_channel[RF][THETA3]
"""servo_channel = ( ( 0,  1,  2),
                     ( 4,  5,  6),
                     ( 8,  9, 10),
                     (12, 13, 14))"""


# This is the list of UIs (User Interfaces) that may be enabled.  Always at least one of these needs to
# be enabled
# NOTE:  The command line arguments processor should always ensure one of this is enabled.
ui_web_gui = False
ui_x11_gui = not ui_web_gui
ui_buttons = False       # set to True only if real hardware is present


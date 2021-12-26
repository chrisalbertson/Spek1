"""
This file holds only CONSTANTS

Our definition of "constant" is a value that does not change after execution begins.
so some code to get the constant values is allowed
"""

# This defines if the real robot hardware is present.
# If not the controller will not attempt to send
# interact with physical device nor will it import the
# associated library modules.
GotHardware = False

body_number_of_legs = 1

shoulder_to_leg_vertical_offset   = -0.015
shoulder_to_leg_horizontal_offset =  0.060

leg_upper_length = 0.110
leg_upper_rotation_limit_min = 180.0
leg_upper_rotation_limit_max =   0.0

leg_lower_length = 0.120
leg_lower_rotation_limit_min = 180.0
leg_lower_rotation_limit_max =   0.0

# We can run one of the two GUIs but as of today not both at once
web_gui = False
x11_gui = not web_gui


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

# on the Spot Micro, the second joint axis does NOT intersect the first but
# is verticall offset about 20 to 15 mm.    In many quadruped models the
# The J1 to J2 distance is just "l1" but SpotMicro needs to constants
j1_to_j2_vertical_offset   = -0.015
j1_to_j2_horizontal_offset =  0.060

leg_upper_length = 0.115

leg_lower_length = 0.130


# We can run one of the two GUIs but as of today not both at once
web_gui = False
x11_gui = not web_gui


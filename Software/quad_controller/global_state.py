import threading

"""This file contains Global Variables that define the real-time state of the robot.

These are needed by many functions and tasks.  They are "read mostly
Watch the locks when globals are shared between tasks
"""

GlobalLock = threading.Lock()

#############################################################
#     WARNING THESE NEED TO BE ACCESSED VIA THE ABOVE LOCK  #
#############################################################
StepLength = 0.0
StepPeriod = 0.0
StepHeight = 0.0
Gait = 'STOP'

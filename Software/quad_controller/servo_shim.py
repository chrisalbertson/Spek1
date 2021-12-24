import math

import logging
log = logging.getLogger(__name__)

import config

if config.GotHardware:
    from adafruit_servokit import ServoKit


class servo_shim:
    """Class to add a thin layer over the physical servo motors.

    The layer adds some features
    1) the servos might be not used at all. This allows testing on computers
       that don't have the hardware to drive servoes
    2) a calibration can be done to make the servos be more linera
    3) logging of commands for post analysis
    4) realtime performance capture
    5) enforment of limits

    This is intened to be a light weight class where all real-time funtions 
    have very low latency
    """

    def __init__(self):

        if config.GotHardware:
            kit = ServoKit(channels=16, frequency=325)

            # Front Left
            kit.servo[ 0].set_pulse_width_range(min_pulse=500, max_pulse=2500)
            kit.servo[ 1].set_pulse_width_range(min_pulse=500, max_pulse=2500)
            kit.servo[ 2].set_pulse_width_range(min_pulse=500, max_pulse=2500)

            # Front right
            kit.servo[ 4].set_pulse_width_range(min_pulse=500, max_pulse=2500)
            kit.servo[ 5].set_pulse_width_range(min_pulse=500, max_pulse=2500)
            kit.servo[ 6].set_pulse_width_range(min_pulse=500, max_pulse=2500)

            # Rear Left
            kit.servo[ 8].set_pulse_width_range(min_pulse=500, max_pulse=2500)
            kit.servo[ 9].set_pulse_width_range(min_pulse=500, max_pulse=2500)
            kit.servo[10].set_pulse_width_range(min_pulse=500, max_pulse=2500)

            # Rear Right
            kit.servo[12].set_pulse_width_range(min_pulse=500, max_pulse=2500)
            kit.servo[13].set_pulse_width_range(min_pulse=500, max_pulse=2500)
            kit.servo[14].set_pulse_width_range(min_pulse=500, max_pulse=2500)

    def set_angle(self, channel_number, radians):


        # FIXME chck that channel number is valid
        # FIXME check that radians is within bounds for this servo
        if (config.GotHardware):
            kit.servo[channel_number].angle = math.degrees(radians)
        else:
            log.debug('set_angle', channel_number, radians)






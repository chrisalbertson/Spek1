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

        # limits set by the 3D printed mechanical structure.  These angles
        # are "theta" after conversion to radians and are relative to the joints
        # not the servos
        self.joint_limit = (
            # Joint1,           Joint2,           Joint3,         unused
            (-360.0, +360.0), (-360.0, +360.0), (-360.0, +360.0), (0, 0), # Front Left
            (-360.0, +360.0), (-360.0, +360.0), (-360.0, +360.0), (0, 0),  # Front Right
            (-360.0, +360.0), (-360.0, +360.0), (-360.0, +360.0), (0, 0),  # Rear Left
            (-360.0, +360.0), (-360.0, +360.0), (-360.0, +360.0), (0, 0))  # Rear Right

        # Set Servo to servo_installed_angle to place joint at theta = 0.0
        self.servo_installed_angle = (
            #   Joint1, Joint2, Joint3, unused
                0.0,    0.0,    0.0,    0.0,    # Front Left
                0.0,    0.0,    0.0,    0.0,    # Front Right
                0.0,    0.0,    0.0,    0.0,    # Rear Left
                0.0,    0.0,    0.0,    0.0)    # Rear Right

        # A servo's effective direction is determined by hw it is installed,
        # if the output spline shaft is facing forward or to the rear, left or right
        # If increasing servo degrees causes increased theta direction = +1
        # If increasing servo degrees causes decreased theta direction = -1
        self.servo_installed_direction = (
            #   Joint1, Joint2, Joint3, unused
                -1,      1,      1,     0,    # Front Left
                -1,     -1,     -1,     0,    # Front Right
                 1,      1,      1,     0,    # Rear Left
                 1,     -1,     -1,     0)    # Rear Right

        if config.GotHardware:
            self.kit = ServoKit(channels=16, frequency=325)

            # Front Left
            self.kit.servo[ 0].set_pulse_width_range(min_pulse=500, max_pulse=2500)
            self.kit.servo[ 0].actuation_range = 180.0
            self.kit.servo[ 1].set_pulse_width_range(min_pulse=500, max_pulse=2500)
            self.kit.servo[ 2].set_pulse_width_range(min_pulse=500, max_pulse=2500)

            # Front right
            self.kit.servo[ 4].set_pulse_width_range(min_pulse=500, max_pulse=2500)
            self.kit.servo[ 5].set_pulse_width_range(min_pulse=500, max_pulse=2500)
            self.kit.servo[ 6].set_pulse_width_range(min_pulse=500, max_pulse=2500)

            # Rear Left
            self.kit.servo[ 8].set_pulse_width_range(min_pulse=500, max_pulse=2500)
            self.kit.servo[ 9].set_pulse_width_range(min_pulse=500, max_pulse=2500)
            self.kit.servo[10].set_pulse_width_range(min_pulse=500, max_pulse=2500)

            # Rear Right
            self.kit.servo[12].set_pulse_width_range(min_pulse=500, max_pulse=2500)
            self.kit.servo[13].set_pulse_width_range(min_pulse=500, max_pulse=2500)
            self.kit.servo[14].set_pulse_width_range(min_pulse=500, max_pulse=2500)

    def set_angle(self, channel_number, radians):
        """Set servo to specified joint angle

        Several things are done here
            1) radians converted to degrees
            2) the joint's mechanical limit is checked and will not be moved past this
            3) the servo range of movment is chaeck and it will not be commanded to
               move past its range.

        Checks #2 and #3 above are releated but independent.  For example a joint might
        allow 240 defrees of movment but we might have a servo with a range of ony 160
        degrees.  Or the servo might be installed with so that not all of its range
        can be used.
        """

        angle_deg = math.degrees(radians)

        # Keep joint angle within joint limits
        joint_min = self.joint_limit[channel_number][0]
        joint_max = self.joint_limit[channel_number][1]
        if not (joint_min <= angle_deg <= joint_max) :
            print('ERROR joint limit', channel_number, angle_deg)
            angle_deg = min(max(joint_min, angle_deg), joint_max)

        servo_deg = (  self.servo_installed_angle[channel_number]
                     + self.servo_installed_direction[channel_number] * angle_deg)

        # Keep servo angle within limits of each servo
        # FIXME -- need to look up limits for each servo
        servo_min = 0.0
        servo_max = 180.0
        if not(servo_min <= servo_deg <= servo_max) :
            print('ERROR servo limit', channel_number, servo_deg)
            servo_deg = min(max(servo_min, angle_deg), servo_max)

        if config.GotHardware:
            self.kit.servo[channel_number].angle = servo_deg
        else:
            log.debug('set_angle', servo_deg)








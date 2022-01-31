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

    def __init__(self, have_hardware: bool = False):
        """Class constructor, builds calibration and limits tables for quick lookup

        Parameters:
            have_hardware:  overides config if set True, used for unit tests
                            when it is hard to change config.  Should not be
                            set outside a test environment.

        """

        # limits set by the robot's mechanical structure.  These angles
        # are "theta" in radians and are relative to the joints
        # FIXME - using dummy values now.
        r_min = -2 * math.pi
        r_max =  2 * math.pi
        self.joint_limit = (
            # Joint1,           Joint2,           Joint3,         unused
            (r_min, r_max), (r_min, r_max), (r_min, r_max), (0, 0), # Front Left
            (r_min, r_max), (r_min, r_max), (r_min, r_max), (0, 0),  # Front Right
            (r_min, r_max), (r_min, r_max), (r_min, r_max), (0, 0),  # Rear Left
            (r_min, r_max), (r_min, r_max), (r_min, r_max), (0, 0))  # Rear Right


        """
        How to Install a Servo
        Use a servo tester of software to set the servo to the 50% point or 1 millisecond
        pulse width point.  Is is the mid point of the servo's range of motion.
        Then place the robot joint at the center point of it's range of motion and install
        the servo into the joint.   Try to get as close as possible but it is 
        nearly impossible
        to perfectly align the tow midpoints.   The spline has 25 teeth so at best you
        can only get to the closest tooth.
        
        After the servo is installed, command it again to it's mid point.  Note the 
        resulting angle of the joint.  This is the number that goes in this table below.
        
        So for example, if joint moves from 100 to 250 degrees it's mid point will be 
        175 degrees.   After you install the servo you notice that the joint movs to 177
        when you command the servo it's midrange point.   You enter 177 in the table below
        so as to record the INSTALLED joint angle at the servo mid point.
        
        Note enter the values in degrees.  We do this because it is a measured value
        ad I don't have a protractor marked in radians.   So the software does the conversion
        for us, later in this init function.
        """
        self.servo_installed_angle = (
            #   Joint1, Joint2, Joint3, unused
                0.0,    0.0,    0.0,    0.0,    # Front Left
                0.0,    0.0,    0.0,    0.0,    # Front Right
              -10.0,    0.0,   90.0,    0.0,    # Rear Left
                0.0,    0.0,    0.0,    0.0)    # Rear Right

       
        # A servo's effective direction is determined by how it is installed,
        # if the output spline shaft is facing forward or to the rear, left or right
        # If increasing servo degrees causes increased theta direction = +1
        # If increasing servo degrees causes decreased theta direction = -1
        # If the servo is not installed set the direction to zero.
        self.servo_installed_direction = (
            #   Joint1, Joint2, Joint3, unused
                -1,      1,      1,     0,    # Front Left
                 1,     -1,     -1,     0,    # Front Right
                -1,     -1,     -1,     0,    # Rear Left
                 1,      1,      1,     0)    # Rear Right

        """
        This table records the measured limits for each servo.  They are all
        different, and should be measured before they are installed.  The table
        is a list of tuples tuples with these feilds:
        (minimum microseconds, maximum microseconds, degrees of movement)
        """
        self.servo_range = (
            # Front Left
            (500, 2500, 180.0),     # Shoulder
            (500, 2500, 180.0),     # upper
            (500, 2500, 180.0),     # lower
            (1,2,3),                # not used

            # Front Right
            (500, 2500, 180.0),     # Shoulder
            (500, 2500, 180.0),     # upper
            (500, 2500, 180.0),     # lower
            (1,2,3),                # not used

            # Rear Left
            (500, 2500, 180.0),     # Shoulder
            (500, 2500, 180.0),     # upper
            (500, 2500, 180.0),     # lower
            (1,2,3),                # not used

            # Rear Right
            (500, 2500, 180.0),     # Shoulder
            (500, 2500, 180.0),     # upper
            (500, 2500, 180.0),     # lower
            (1,2,3),                # not used
        )

	# Set this depending on the servo specs.   Cheap servos use 50Hz
	# better servos ue 300+ Hz
        pwm_freq = 50

        if config.GotHardware or have_hardware:
            self.kit = ServoKit(channels=16, frequency=pwm_freq)
            for chan in range(16):
                self.kit.servo[chan].set_pulse_width_range(
                    min_pulse=self.servo_range[chan][0],
                    max_pulse=self.servo_range[chan][1])
                self.kit.servo[chan].actuation_range = self.servo_range[chan][2]

        """
        Below we do some pre-computation so as to save a tiny amount of processing later
        when running real-time.
        The relationship between radians on the robot frame and degrees in the servo frame
        is linear so we find the two constants "m" and "b" for a linear equation of
        the form "y = mx + b".
        """
        deg_per_radian = 180.0 / math.pi
        t2s = []    #set to empty list
        for chan in range(16):

            mid  = self.servo_range[chan][2]/2.0
            inst = self.servo_installed_angle[chan]
            direction = self.servo_installed_direction[chan]

            if direction == +1:
                mult = deg_per_radian
                bias = mid - inst
            elif direction == -1:
                mult = -deg_per_radian
                bias = mid + inst
            else:
                mult = 0
                bias = 0

            t2s.append((mult, bias))    # append a tuple

        # save it as a tuple, tuples are a little faster than lists.
        # and we need to do maybe 600 lookups per second
        self.theta2servo = tuple(t2s)

    def set_angle(self, channel_number: int, radians: float) -> None:
        """Set servo to specified joint angle

        Several things are done here
            1) radians converted to degrees
            2) the joint's mechanical limit is checked and will not be moved past this.
               The servo's range of motion limit is baked into the joint limit by the
               __init__() function, so we only have to check once here.
            3) check if real hardware is present either move the servo or write to a log.
        """

        # Keep joint angle within joint limits
        joint_min = self.joint_limit[channel_number][0]
        joint_max = self.joint_limit[channel_number][1]
        if not (joint_min <= radians <= joint_max):
            print('ERROR joint limit', channel_number, radians)
            radians = min(max(joint_min, radians), joint_max)

        m = self.theta2servo[channel_number][0]
        b = self.theta2servo[channel_number][1]
        servo_deg = ( m * radians + b)

        if config.GotHardware:
            print("SERVO", channel_number, servo_deg, "--", m, b)
            self.kit.servo[channel_number].angle = servo_deg
        else:
            pass
            # log.debug('angle ' + str(servo_deg) + ' chan ' + str(channel_number))
        return


    def set_raw_degrees(self, channel_number: int, degrees: float) -> None:
        """
        Args:
            channel_number: An integer in thranf 0...15 that i passed to servokit
            degrees:        The angle that the servo is to go to, passed
                            unchanged to the servo

        Returns:   None.


        This function should only be called during off-line calibration, not real-time,
        so it is OK to let it crash if the parameters are out of range
        """

        assert 0 =< channel_number =< 15
        assert 0.0 =< degrees =< 360.0
        
        self.kit.servo[channel_number].angle = degrees
        return


if __name__ == "__main__":

    logging.basicConfig(filename='quad_controller.log',
                        filemode='w',
                        level=logging.DEBUG, )
    servo = servo_shim(have_hardware=True)

    servo.set_angle(16, )
import math
import matplotlib.pyplot as plt
import logging
#from adafruit_servokit import ServoKit
import config
from piecewise_linear import PiecewiseLinear


class Leg:
    """Leg class represents the physical leg on a robot

    We assume the ground is flat, and we handle only one foot.
    Y is runs fore and aft and is positive in the direction the robot is moving.
    The origin is relative to the start of the step
    Z is up.
    X is to the left but is of no concern here

    Phase is the fraction of a step that is complete.  The range is 0.0 -> 1.0
    Phase == 0.0 is defined as the neutral standing position of the foot which is
    typically directly under the hip.
    """

    def __init__(self):
        """Constructor for the basic foot motion generator

        :param: air_fraction: The fraction of a step where the foot is not in ground contact, range is 0..1
        :param: step_length:  The length of one step in meters
        """

        """
        The foot path is defined by a pair of functions that give points in
        (Y,Z) as a funtion of time.  All numbers are scaled to one unit.
    
        So the foot location is the point (f(t), g(t)) where t is the
        faction of the foot's "air time".  While "air time" is some
        faction of the step period.
    
        For now these curves are defined by simple constant shape curves
        later we can add dynamic curve generation
        """

        self.len_upper = config.leg_upper_length
        self.len_lower = config.leg_lower_length
        self.limit_leg_reach = self.len_lower + self.len_upper

        # fixme use globals set by GUI
        # self.step_length = step_length
        self.step_length = 0.050
        self.step_height = 0.04  # height of step at maximum TBD make this a parameter
        # fixme use globals set by GUI
        # self.air_fraction = air_fraction
        self.air_fraction = 0.20
        # for testing, we can use a triangle ramp footpath.  Later we can use
        # more realistic curves
        self.footpathZ = PiecewiseLinear(((0.0, 0.0),
                                          (0.25, 0.9),
                                          (0.5, 1.0),
                                          (0.75, 0.9),
                                          (1.0, 0.0)))

        self.footpathY = PiecewiseLinear(((0.0, 0.0),
                                          (0.25, 0.18),
                                          (0.5, 0.5),
                                          (0.75, 0.82),
                                          (1.0, 1.0)))


        # leg_phase is a number in the range 0..1 that says where in a leg cycle a leg is.
        # This is different from "step_phase" as step_phase is in reference to the entire robot
        # by convention a leg cycle always starts when the leg lifts from the ground.
        # But in practice robots and animals start from a nuetral position.  So strides rarely
        # start at phase==0.
        # Assume step is centered around the hip.  The leg moves equal distance to the front and rear of the hip.

    def get_legYZ(self, leg_phase):
        """Generate the foot locations on an YZ plane that is aligned to the direction of movement

        :param leg_phase: A number in the range 0...1 that is the phase of the leg cycle.
        :return:

        >>> l = Leg_YZ(1.0, 1.0)
        >>> l.get_legYZ(0.5)
        (False, (0.0, 0.5))
        """

        # returns
        #   Contact,            boolean, True if foot is in ground contact.
        #   (y_pos, z_pos),     foot location
        #                           Y is meters forward of start of the step.  In other words.
        #                             the foot lifts off the ground at Y=0.
        #                           Z is metres above the ground

        # Is the foot on the ground or in the air?
        # By convention a leg cycle stars as the fot lifts off the gound.   S if the
        # "air fraction" is 0.20 the foot lands at leg_phase == 0.20.
        if leg_phase > self.air_fraction:
            contact = True
            # the foot is in contact with the ground.
            return contact, (self.step_length, 0.0)
        else:
            contact = False

            # The foot is in the air.  We need to look in the trajectory table
            # The table records a function with range 0..1  We map this interval to the
            # "air fraction"
            fraction_air_fraction = leg_phase / self.air_fraction
            z = self.step_height * self.footpathZ.get_point(fraction_air_fraction)
            y = self.step_length * self.footpathY.get_point(fraction_air_fraction)
            return contact, (y, z)


    def distance(self, p1, p2):
        (x1, y1) = p1
        (x2, y2) = p2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


    def angle_SSS(self, a, b, c):
        """Apply Law of Cosines to get A given abc

        The Law of Cosines states that
            a^2 = b^2 + c^2 - 2*b*c * cos(A)
        """
        logging.debug('sss', a, b, c)

        # FIXME
        if (a == 0.0 or b == 0.0 or c == 0.0):
            return 0.0

        # Return the angle in radians
        return math.acos((a ** 2 - b ** 2 - c ** 2) / (-2.0 * b * c))


    def ik2d(self, foot_pt):
        """return two joint angles given hip-relative foot position"""

        # hip, knee and foot locations define a triangle.
        # Call the vertexes H, K, F.
        # Call the opposite sides h, k, f.
        # So the sides are f=HK, h=FK and k=HF
        # We know the lengts of the two leg segments from the CD models.
        lenHK = self.len_upper
        lenKF = self.len_lower

        # Get the distance from hip to foot
        hip_pt = (0.0, 0.0)
        lenHF = self.distance(hip_pt, foot_pt)

        # check if foot_pt is close enough to the hip to be reached,
        # leg fully extended.
        if (lenHF > self.limit_leg_reach):
            print("leg reach limit", self.limit_leg_reach, self.len_lower, self.len_upper)
            raise ValueError("leg reach limit")

        # We can use the law of cosines to get the knee angle
        angleK = self.angle_SSS(lenHF, lenHK, lenKF)

        # Find angle at hip from knee to foot
        angleH = self.angle_SSS(lenKF, lenHF, lenHK, )

        # Find the angle at hip from Y-axis to foot
        if (foot_pt[0] > 0.0):
            angleF = math.asin(foot_pt[1] / lenHF)
        else:
            angleF = math.pi - math.asin(foot_pt[1] / lenHF)

        print("lenHF=", lenHF,
              "angleF=", angleF, math.degrees(angleF),
              "angleK=", angleK, math.degrees(angleK),
              "angleH=", angleH, math.degrees(angleH))

        return angleK, angleF + angleH


    def plotLeg(self, footPt):
        """ Draws a plot to scale of a leg with the foot in the specified position
        """
        angleK, angleH = self.ik2d(footPt)
        # Find knee point
        kneeY = self.len_upper * math.cos(angleH)
        kneeZ = self.len_upper * math.sin(angleH)

        # plt.rcParams["figure.figsize"] = [2.0, 2.0]
        # plt.rcParams["figure.autolayout"] = True
        hip = [0.0, 0.0]

        x_values = [hip[0], kneeY, footPt[0]]
        y_values = [hip[1], kneeZ, footPt[1]]
        plt.axis('equal')
        plt.plot(x_values, y_values,
                 'bo', linestyle="--",
                 scalex=True, scaley=True, )
        # plt.text(point1[0] - 0.015, point1[1] + 0.25, "Point1")
        # plt.text(point2[0] - 0.050, point2[1] - 0.25, "Point2")

        # Sanity check
        print(
            "upper len", self.distance(hip, (kneeY, kneeZ)),
            "lower len", self.distance((kneeY, kneeZ), footPt)
        )
        plt.show()


if __name__ == "__main__":
    import doctest
    doctest.testmod()

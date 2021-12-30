import math
import matplotlib.pyplot as plt
import logging
import config
from piecewise_linear import PiecewiseLinear
import global_state as gs


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
        (Y,Z) as a function of time.  All numbers are scaled to one unit.
    
        So the foot location is the point (f(t), g(t)) where t is the
        faction of the foot's "air time".  While "air time" is some
        faction of the step period.
    
        For now these curves are defined by simple constant shape curves
        later we can add dynamic curve generation
        """

        # grab globals.  These value can change in real time
        gs.GlobalLock.acquire()
        # self.step_period = gs.StepPeriod
        # self.step_length = gs.StepLength
        # self.step_height = gs.StepHeight
        self.gait =        gs.Gait
        self.air_fraction = gs.StepAirFraction
        gs.GlobalLock.release()

        # grab configuration.  These value
        self.len_upper = config.leg_upper_length
        self.len_lower = config.leg_lower_length
        self.limit_leg_reach = self.len_lower + self.len_upper

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
        # But in practice robots and animals start from a neutral position.  So strides rarely
        # start at phase==0.
        #
        # Example foot motion:
        #   1. At phase = 0.0 the foot is still at Y=0 and Z=0, but it is abot the lift off
        #   2. At phase = 0.01 the foot is just off the ground and moving forward.
        #      Both Y and Z are tiny positive numbers
        #   3. When the leg phase equals the air-fraction the foot is just landing after
        #      moving forward onr step length.
        #   4. After the foot has landed it remains stationary and fixed to the ground
        #      for the rest of the leg cycle.

        # The leg moves equal distance to the front and rear of the hip.

    def get_legYZ(self, leg_phase):
        """Generate the foot locations on an YZ plane that is aligned to the direction of movement

        :param leg_phase: A number in the range 0...1 that is the phase of the leg cycle.
        :return:

        >>> l = Leg()
        >>> l.get_legYZ(0.0)
        (False, (0.0, 0.5))
        >>> l = Leg()
        >>> l.get_legYZ(0.5)
        (False, (0.0, 0.5))
        >>> l = Leg()
        >>> l.get_legYZ(9.9)
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
            return contact, (1.0, 0.0)
        else:
            contact = False

            # The foot is in the air.  We need to look in the trajectory table
            # The table records a function with range 0..1  We map this interval to the
            # "air fraction"
            fraction_air_fraction = leg_phase / self.air_fraction
            z = self.footpathZ.get_point(fraction_air_fraction)
            y = self.footpathY.get_point(fraction_air_fraction)
            return contact, (y, z)


    def distance(self, p1, p2):
        (x1, y1) = p1
        (x2, y2) = p2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def distance3d(self, p1, p2):
        (x1, y1, z1) = p1
        (x2, y2, z2) = p2
        return math.sqrt(  (x2 - x1) ** 2
                         + (y2 - y1) ** 2
                         + (z2 - z1) ** 2)


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

    def ik3d(self, x, y, z):
        """ Compute angles from x,y,z position of foot.

        According to the ROS REP03 standard,
            X - the positive x means forward,
            Y - the positive y means left
            Z - positive z means up.



        F=Length of shoulder-point to target-point on x/y only
        G=length we need to reach to the point on x/y
        H=3-Dimensional length we need to reach
        """

        # fixme - these need to come from config
        l1 = 0.050
        l2 = 0.015
        l3 = 0.110
        l4 = 0.130

        F = math.sqrt(y ** 2 + z ** 2 - l1 ** 2)
        G = F - l2
        H = math.sqrt(G ** 2 + x ** 2)

        theta1 = -math.atan2(z, y) - math.atan2(F, -l1)

        D = (H ** 2 - l3 ** 2 - l4 ** 2) / (2 * l3 * l4)
        theta3 = math.acos(D)

        theta2 = math.atan2(x, G) - math.atan2(l4 * math.sin(theta3), l3 + l4 * math.cos(theta3))

        print("angles", theta1, theta2, theta3)
        print("degrees", math.degrees(theta1), math.degrees(theta2), math.degrees(theta3))
        return (theta1, theta2, theta3)

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
            print("ERROR leg-reach limit", self.limit_leg_reach, self.len_lower, self.len_upper)
            # raise ValueError("leg reach limit")

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

""" Top level class that defines the robot object and some actions the robot can perform
"""

import logging
log = logging.getLogger(__name__)

import leg_v2 as leg
from enum import Enum

import config

# This is used to implement a very simple state machine
class WalkingState (Enum):
    AT_REST  = 0
    STOPPING = 1
    STARTING = 2
    WALKING  = 3

# fixme this needs to be run from a thread
# fixme "velocity" needs to be a vector, not a scaler, so the robot can side-step.
def walking(velocity: float):

    legs = leg.Legs

    # Set the leg phases and "air fraction" to define an amble gait
    step_air_fraction = 0.20
    leg_relative_phase = [0, 0, 0, 0]
    leg_relative_phase[leg.LegId.LF] =  0.0
    leg_relative_phase[leg.LegId.RR] = 25.0
    leg_relative_phase[leg.LegId.RF] = 50.0
    leg_relative_phase[leg.LegId.LR] = 75.0


    while True:

        # get the current step phase
        step_phase = (step_start - time.time) / step_period

        for indx in range(4):
            leg_phase = step_phase - leg_relative_phase[indx]
            if leg_phase < 0:
                leg_phase = 1.0 + leg_phase

            # is this leg in ground contact?
            # Feet always start to lift off the ground when phase is zero
            # and remain off the ground until phase is > then the current air fraction
            if leg_phase < step_air_fraction:
                # This foot is in the air, So we must use a table look-up
                # to gets its current position.  First we compute "air_fraction"
                # whish is the fraction of the air path completed.
                air_fraction =
                foot_point = foot_air_path(air_fraction)

            else:
                # This foot is in contact with the ground.  So we know it is stationary
                # relative to the ground and moves relative to the robot exactly opposite
                # to the robot's velocity
                x_next = x_last - (velocity * loop_period)

            Lp[indx] = [x_next, y_next, z_next]

        # fixme -- add in any body position and ange needed for things like balance

        # Find the joint angles to move the foot to the required position
        joint_angles = kinematics.calcIK(Lp,  (0, 0, 0), (0, 0, 0))

        # Set the robots joints to those we computed.
        # Note that the function below will check if the motors are enabled or if
        # we are running in simulation only
        for indx in range(4)
            legs[indx].move_to_angle(joint_angles[indx])


class Robot:

    def __init__(self):

        self.legs = leg_v2.Legs()

        self.walking_state = WalkingState.AT_REST
        return

    def stop_natural(self):
        """ Stops the robot at the next natural opportunity. """

        if  (self.walking_state == WalkingState.AT_REST or
             self.walking_state == WalkingState.STOPPING):
            # The robot is already stopping or stopped, so there is nothing to do
            pass
        elif (self.walking_state == WalkingState.STARTING or
              self.walking_state == WalkingState.WALKING):
            # set the state to "STOPPING" and the walk thread will notice
            # this and stop next time it has all four feet on the ground
            self.walking_state = WalkingState.STOPPING
            return

        else:
            log.error("invalid state " + str(self.walking_state))
            return


    def stop_panic(self):
        """Stops robot as quickly as possible.  Jams feet to low wide stance"""
        # fixme -- Need to actual do a panic stop
        self.stop_natural()
        return

    # FIXME -- This needs to run inside a thread
    def walk(self, velocity: float, ):
        """ The robot walks"""

        # Check the current state.
        if self.walking_state == WalkingState.AT_REST:
            # Start the walking process
            ## FIXME START THREAD
            walking(velocity)

        elif (self.walking_state == WalkingState.WALKING or
              self.walking_state == WalkingState.STARTING):
            # The robot is already walking so there is nothing to do
            return

        elif self.walking_state == WalkingState.STOPPING:
            # Cancel the Stopping state and reset it to starting
            self.walking_state = WalkingState.STARTING
            return

        else:
            log.error("invalid state " + str(self.walking_state))
            return

    def set_body_angles_relative(self, angles: (float, float, float)):
        pass

    def set_body_center_relative(self, body_point: (float, float, float)):
        pass

    def set_stance_width_relative(self, stance_width: float):
        pass

    def set_stance_length_relative(self, stance_length: float):
        pass



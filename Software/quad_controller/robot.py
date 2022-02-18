""" Top level class that defines the robot object and some actions the robot can perform
"""
import time
import threading
import logging
import math
import numpy as np
import sm_kinematics

log = logging.getLogger(__name__)

###import leg_v2 as leg
from enum import Enum

import config
import air_path
import joints

# We always count and order the legs this way, left to right and front to rear
class LegId (Enum):
    LF = 0      # Left Front is first and number 0
    RF = 1
    LR = 2
    RR = 3      # Right Rear is last and number 3

# This is used to implement a very simple state machine
class WalkingState (Enum):
    AT_REST  = 0
    STOPPING = 1
    STARTING = 2
    WALKING  = 3

init_time = time.time()

## GLOBAL VARIABLES ####################
gbl_velocity = 0.0
gbl_walking_state = WalkingState.AT_REST
run_walking_control_loop = False

gbl_body_center_up =      0.180
gbl_body_center_left =    0.0
gbl_body_center_forward = 0.0

walk_parameter_lock = threading.Lock()
## END GLOBAL VERIABLES #################


def limit_check_lp(Lp, check_id=''):
    """Perform a diagnostic error check on Lp array -- remove this in production code"""
    if abs(np.amax(Lp[:, :3])) > 0.300:

        # ERROR something is wrong, the legs are not over 300 mm long
        log.error('Lp array limit check failed')
        print(check_id, Lp)
    return

def get_neural_stance_Lp() -> np.array:
    """Function to return a reasonable neutral stance on flat ground"""

    # Units for these are Meters.
    vertical =      0.180
    stance_width =  0.150
    stance_length = 0.260

    stance_halfwidth = stance_width / 2.0
    stance_halflength = stance_length / 2.0

    # Distances of each foot from body center
    #                   fore/aft         height       left/right
    Lp = np.array([[ stance_halflength, -vertical,  stance_halfwidth, 1],   # LF
                   [ stance_halflength, -vertical, -stance_halfwidth, 1],   # RF
                   [-stance_halflength, -vertical,  stance_halfwidth, 1],   # LR
                   [-stance_halflength, -vertical, -stance_halfwidth, 1]])  # RR

    limit_check_lp(Lp, check_id='get_neural_stance_Lp')
    return Lp

def get_length_period(velocity: float) -> (float, float):
    """ find combination of step length and period given a desired velocity

    This is a placeholder function that returns a fixed workable result.
    It needs to be replaced with one that works like this

    First find a step length based on velocity.  This will be a lookup.  The lookup
    function will strongly prefer some length and only move away from that fo very
    fast or slow speeds.   There will be a different look with smaller length for
    sideways or backward walking.   For arbitrary angles we mix these with sin/cos.

    Next step period is computed to give the desired velocity with the preferred
    step length

    Later maybe so other factors will be added to the calculation like if the
    ground is flat or inclined
    """

    max_velocity = 0.1      # this is as fast as the robot can move in meters per second
    preferred_step_length = 0.050
    step_period = preferred_step_length / min(velocity, max_velocity)

    return preferred_step_length, step_period


def phase_duration_4_foot_contact(phase_list, air_fraction) -> float:

    not_zero = []
    for phase in phase_list:
        if phase != 0.0:
            not_zero.append(abs(phase))

    smallest_not_zero = min(not_zero)
    contact_duration = smallest_not_zero - air_fraction

    assert contact_duration > 0.0, 'snz={0}, af={1}'.format(smallest_not_zero, air_fraction)
    return contact_duration


def log_foot(Lp, foot_idx, phase, t):
    global init_time

    log.info('log_foot {0:7.3}, {1:7.3}, {2:s}'.format(t-init_time, phase, str(Lp[foot_idx])))
    return

# fixme "velocity" needs to be a vector, not a scaler, so the robot can side-step.
def walking():

    global run_walking_control_loop
    global walk_parameter_lock
    global gbl_walking_state

    global gbl_body_center_up
    global gbl_body_center_left
    global gbl_body_center_forward

    ### Channel number on PCA9685 for each angle
    #                               th1 th2 th3
    servo_channel = np.array([[ 0,  1,  2],    # LF
                              [ 4,  5,  6],    # RF
                              [ 8,  9, 10],    # LR
                              [12, 13, 14]],    # RR
                             dtype=np.uint8)
    servos = joints.ServoShim()
    ap = air_path.AirPath()
    smk = sm_kinematics.Kinematic()

    # numpy array to hold position of all four feet
    Lp = get_neural_stance_Lp()
    limit_check_lp(Lp, check_id='stance')

    # Set the leg phases and "air fraction" to define an amble gait
    # where one legs moves forward at a time
    # In this gait, leg LF leads and the others follow
    step_air_fraction = 0.20
    leg_relative_phase = [0., 0., 0., 0.]
    leg_relative_phase[LegId.LF.value] =  0.00
    leg_relative_phase[LegId.RR.value] = -0.25
    leg_relative_phase[LegId.RF.value] = -0.50
    leg_relative_phase[LegId.LR.value] = -0.75

    # Get Velocity, so we can compute initial step length and period
    walk_parameter_lock.acquire()
    velocity = gbl_velocity
    body_center_up =      gbl_body_center_up
    body_center_left =    gbl_body_center_left
    body_center_forward = gbl_body_center_forward
    walk_parameter_lock.release()

    step_height = 0.015 #FIXME This should not be a constant
    step_length, step_period = get_length_period(velocity)
    last_velocity = velocity

    # Velocity should be a vector, not a scaler as this robot can
    # walk in a direction it is not facing.
    velocity_vector = np.array([velocity, 0.0, 0.0])     # [forward, up left]

    heartbeat_time = 0.0
    heartbeat_period = 1.0

    loop_frequency = 10.0  # fixme this should come from config
    loop_period = 1.0 / loop_frequency
    step_start = 0

    while True:
        tick_start = time.time()

        # copy global parameters
        log.debug('acquiring the lock...')
        if walk_parameter_lock.acquire(blocking=True, timeout=0.010):
            run = run_walking_control_loop
            walking_state    = gbl_walking_state
            velocity = gbl_velocity
            body_center_up =      gbl_body_center_up
            body_center_left =    gbl_body_center_left
            body_center_forward = gbl_body_center_forward
            walk_parameter_lock.release()
        else:
            log.error('>>>failed to acquire lock<<<  {0}'.format(tick_start))

        # Check if this thread should continue running
        if not run:
            log.info('walk_thread terminating at {0}'.format(tick_start))
            break

        # Heartbeat -- show the world this thread is alive
        if tick_start > heartbeat_time:
            # TBD need to blink a LED
            log.debug('heartbeat at {0}'.format(tick_start))
            heartbeat_time = tick_start + heartbeat_period

        # Check the state.  If AT_REST then we should not advance the step_state
        # or move the legs other than to maintain balance and body position
        if walking_state != WalkingState.AT_REST:

            # get the current step phase, handle wrap-around.  Phase can never
            # be greater than 1.0
            # Note that step_start is initialized to zero when ever the robot
            # is AT_REST, so we set it the first time around
            if step_start == 0.0:
                # fixme - find a besst way than what is commented out below
                # We do nt start walking at phase = 0.0.   We start before then
                # at the point in the cycle where all the fet just made ground
                # contact.   So at start-up there is a delay before the first
                # foot leaves the ground
                '''
                phase_duration = phase_duration_4_foot_contact(leg_relative_phase,
                                                                 step_air_fraction)
                time_duration = phase_duration * step_period
                step_start = tick_start - time_duration
                step_phase = 1.0 - phase_duration
                '''
                step_start = tick_start
                step_phase = 0.0
            else:
                step_phase = (tick_start - step_start) / step_period
                if step_phase > 1.0:
                    (step_phase, _) = math.modf(step_phase)
                    step_start = tick_start - (step_period * step_phase)

            assert step_phase <= 1.0, \
                   '{0}, {1}, {2}'.format(tick_start, step_start, step_period)

            # Track the number of feet on the ground.  We start assuming none are
            num_feet_on_ground = 0

            for leg_id in LegId:
                leg_phase = step_phase + leg_relative_phase[leg_id.value]

                if leg_phase < 0:
                    leg_phase = 1.0 + leg_phase
                assert 0.0 <= leg_phase <= 1.0
                log.debug('leg = {0}, leg_phase = {1}, time = {2}'.
                          format(leg_id.value, leg_phase, tick_start))

                # is this leg in ground contact?
                # Feet always start to lift off the ground when phase is zero
                # and remain off the ground until phase is > then the current air fraction
                if leg_phase < step_air_fraction:
                    ### This foot is in the air ###
                    # So we must use a table look-up
                    # and some math to get its current position.
                    # First we compute "air_fraction"
                    # which is the fraction of the air path completed.
                    air_fraction_completed = leg_phase / step_air_fraction
                    assert 0.0 <= air_fraction_completed <= 1.0

                    (foot_x, foot_z) = ap.get_xz(air_fraction_completed,
                                                 step_length, step_height)
                    #fixme - the below assume the foot moves only fore/aft and the robot
                    #        never side steps.  We should project X on to x,y plane
                    Lp[leg_id.value, :2] = [foot_x, foot_z - body_center_up]
                    # DEBUGGING ONLY
                    # limit_check_lp(Lp, check_id='air')

                else:
                    ### This foot is in contact with the ground. ###
                    num_feet_on_ground += 1

                    # We know this foot is stationary relative to the ground
                    # abd moves relative to the robot exactly opposite
                    # to the robot's velocity vector. So we move the foot
                    # by the -1 times amount the robot moves in one loop period
                    Lp[leg_id.value, 0:3] -= velocity_vector * loop_period
                    Lp[leg_id.value, 1]   = -body_center_up
                    # DEBUGGING ONLY
                    # limit_check_lp(Lp, check_id='ground')
        else:
            num_feet_on_ground = 4      # When AT_REST we assume 4 feet on ground
            step_start = 0              # Step phase does not advance when AT_REST

        # Check if all four feet are on the ground, if so then we can
        # update any of the step parameters
        if num_feet_on_ground == 4:

            # If the walk state was "stopping" now is the time to actually stop
            # fixme -- Here, we instantly change walking stat, assuming this is OK
            #          while all four feet are on the ground.  Maybe we should do
            #          a acceleration/deceleration ramp.  maybe the first step
            #          from a stop needs to be 1/2 step_length?
            if walking_state == WalkingState.STOPPING:
                # Setting to AT_REST causes the step phase to stop advancing and
                # the step start time to reset.
                walking_state = WalkingState.AT_REST

            elif walking_state == WalkingState.STARTING:
                walking_state = WalkingState.WALKING

            # Now is the time, with all four feet down
            # to adjust the step parameters.
            if velocity != last_velocity:
                step_length, step_period = get_length_period(velocity)
                last_velocity = velocity
                # Velocity should be a vector, not a scaler as this robot can
                # walk in a direction it is not facing.
                velocity_vector[0] = velocity  # [forward, up left]

        # fixme -- add in any body position and angle needed for things like balance

        # Find the joint angles to move the foot to the required position
        body_angles = (0.0, 0.0, 0.0)
        body_center = (0.0, 0.0, 0.0)
        # DEBUGGING ONLY
        # limit_check_lp(Lp, check_id='ik')
        lg = 0 # leg to log
        log_foot(Lp, lg, step_phase, tick_start)

        joint_angles = smk.calcIK(Lp, body_angles, body_center)
        log.debug('log_foot                 angles {0:7.3f}, {1:7.3f}, {2:7.3f}'.
                  format(math.degrees(joint_angles[lg, 0]),
                         math.degrees(joint_angles[lg, 1]),
                         math.degrees(joint_angles[lg, 2])))

        # Set the robots joints to the angles computed above.
        # Note that the function below will check if the motors are enabled or if
        # we are running in simulation only.  So maybe the angles only go to a log
        # file, or they could move the physical robot.
        for leg_idx, leg in enumerate(joint_angles):
            for theta_idx, theta in enumerate(leg):
                servos.set_angle(servo_channel[leg_idx, theta_idx], theta)

        tick_elapsed = time.time() - tick_start
        sleep_time = loop_period - tick_elapsed

        if sleep_time < 0.001:
            log.warning('loop overrun, elapsed = {0}'.format(tick_elapsed))
        else:
            assert sleep_time < loop_period
            assert sleep_time > 0.001
            time.sleep(sleep_time)

    # this task ends
    log.error('>>>>>>>>>> walk_thread hits return <<<<<<<<<<<')
    return

walk_thread = threading.Thread(target=walking, args=(), daemon=True)

class Robot:

    def __init__(self):

        self.walking_state = WalkingState.AT_REST
        return

    def kill(self):
        global walk_thread
        global walk_parameter_lock
        global run_walking_control_loop

        log.debug('kill entered at {0}'.format(time.time()))

        walk_parameter_lock.acquire()
        run_walking_control_loop = False
        walk_parameter_lock.release()
        return

    def stop_natural(self):
        """ Stops the robot at the next natural opportunity. """
        log.debug('stop_natural entered at {0}'.format(time.time()))
        # fixme -- need to lock globals
        if  (self.walking_state == WalkingState.AT_REST or
             self.walking_state == WalkingState.STOPPING):
            # The robot is already stopping or stopped, so there is nothing to do
            pass
        elif (self.walking_state == WalkingState.STARTING or
              self.walking_state == WalkingState.WALKING):
            # set the state to "STOPPING" and the walk thread will notice
            # this and stop next time it has all four feet on the ground
            self.walking_state = WalkingState.STOPPING
        else:
            log.error("invalid state " + str(self.walking_state))

        log.debug('stop_natural returning')
        return

    def stop_panic(self):
        """Stops robot as quickly as possible.  Jams feet to low wide stance"""
        self.walking_state = WalkingState.AT_REST
        return

    def set_velocity(self, velocity: float):
        """Sets global velocity variable"""

        global walk_parameter_lock

        walk_parameter_lock.acquire()
        gbl_velocity = velocity
        walk_parameter_lock.release()
        return

    def walk(self, velocity: float):
        """ The robot walks"""

        global run_walking_control_loop
        global gbl_walking_state
        global gbl_velocity
        global walk_parameter_lock

        log.debug('walk entered at {0}'.format(time.time()))

        # Check the current state.
        walk_parameter_lock.acquire()
        self.walking_state = gbl_walking_state
        gbl_velocity = velocity
        walk_parameter_lock.release()


        if self.walking_state != WalkingState.WALKING:

            walk_parameter_lock.acquire()
            gbl_walking_state  = WalkingState.STARTING
            run_walking_control_loop = True
            walk_parameter_lock.release()
            self.walking_state = WalkingState.STARTING

            # Start the walking thread if not already started
            if not walk_thread.is_alive():
                log.debug("starting walk_thread")
                walk_thread.start()
            # FIXME -- when to join ith this thread?
        log.debug("walk returning")
        return

    def set_body_angles_relative(self, angles: (float, float, float)):
        pass

    def set_body_center_relative(self, body_point: (float, float, float)):
        pass

    def set_stance_width_relative(self, stance_width: float):
        pass

    def set_stance_length_relative(self, stance_length: float):
        pass

def walk_test():
    r = Robot()
    r.walk(0.05)
    time.sleep(10)
    r.stop_natural()
    r.kill()
    return



if __name__ == "__main__":

    logging.basicConfig(filename='quad_controller.log',
                        filemode='w',
                        level=logging.DEBUG, )

    walk_test()
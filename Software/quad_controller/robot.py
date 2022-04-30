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
import balance

#FIXME
#import imu
#imu = imu.IMU()

# We always count and order the legs this way, left to right and front to rear
class LegId (Enum):
    LF = 0      # Left Front is first and number 0
    RF = 1
    LR = 2
    RR = 3      # Right Rear is last and number 3

init_time = time.time()

## GLOBAL VARIABLES ####################
gbl_velocity = 0.0
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


def neutral_stance_Lp() -> np.array:
    """Function to return a reasonable neutral stance on flat ground"""

    # Units for these are Meters.
    vertical =      0.180
    stance_width =  0.120
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

    max_velocity = config.max_forward_speed      # meters per second
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

    global gbl_body_center_up
    global gbl_body_center_left
    global gbl_body_center_forward

    ### Channel number on PCA9685 for each angle
    #                         th1 th2 th3
    servo_channel = np.array([[ 0,  1,  2],    # LF
                              [ 4,  5,  6],    # RF
                              [ 8,  9, 10],    # LR
                              [12, 13, 14]],   # RR
                             dtype=np.uint8)
    servos = joints.ServoShim()
    ap = air_path.AirPath()
    smk = sm_kinematics.Kinematic()

    # numpy array to hold position of all four feet
    Lp_neutral = neutral_stance_Lp()
    Lp = Lp_neutral.copy()
    limit_check_lp(Lp, check_id='stance')

    # Set the leg phases and "air fraction" to define an amble gait
    # where one legs moves forward at a time
    # In this gait, leg LF leads and the others follow
    step_air_fraction = 0.125
    leg_relative_phase = [0., 0., 0., 0.]
    gait = 'trot'

    if gait == 'trot':
        leg_relative_phase[LegId.LF.value] =  0.00
        leg_relative_phase[LegId.RR.value] = -0.05
        leg_relative_phase[LegId.RF.value] = -0.50
        leg_relative_phase[LegId.LR.value] = -0.55
    elif gait == 'creep':
        leg_relative_phase[LegId.LF.value] =  0.00
        leg_relative_phase[LegId.RR.value] = -0.25
        leg_relative_phase[LegId.RF.value] = -0.50
        leg_relative_phase[LegId.LR.value] = -0.75
    else:
        log.error('invalid gait')

    # Get Velocity, so we can compute initial step length and period
    with walk_parameter_lock:
        velocity = gbl_velocity
        body_center_up =      gbl_body_center_up
        body_center_left =    gbl_body_center_left
        body_center_forward = gbl_body_center_forward

    if velocity == 0.0:
        step_length = 0.0
        step_period = 0.0
        standing = True
    else:
        step_length, step_period = get_length_period(velocity)
        standing = False

    step_height = 0.024 #FIXME This should not be a constant
    last_velocity = velocity

    # Velocity should be a vector, not a scaler as this robot can
    # walk in a direction it is not facing.
    velocity_vector = np.array([velocity, 0.0, 0.0])     # [forward, up left]

    heartbeat_time = 0.0
    heartbeat_period = 1.0

    loop_frequency = 20.0  # fixme this should come from config
    loop_period = 1.0 / loop_frequency
    step_start = 0.

    while True:
        tick_start = time.time()

        # copy global parameters
        log.debug('acquiring the lock...')
        if walk_parameter_lock.acquire(blocking=True, timeout=0.010):
            run = run_walking_control_loop
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

        # Check the state.  If at rest then we should not advance the step_state
        # or move the legs other than to maintain balance and body position
        if not standing:

            # get the current step phase, handle wrap-around.  Phase can never
            # be greater than 1.0
            # Note that step_start is initialized to zero when ever the robot
            # is AT_REST, so we set it the first time around
            if step_start == 0.0:
                step_start = tick_start
                step_phase = 0.0
            else:
                step_phase = (tick_start - step_start) / step_period
                if step_phase > 1.0:
                    (step_phase, _) = math.modf(step_phase)
                    step_start = tick_start - (step_period * step_phase)

            assert step_phase <= 1.0, \
                   '{0}, {1}, {2}'.format(tick_start, step_start, step_period)

            # Track the number of feet on the ground.  We start by assuming none are
            num_feet_on_ground = 0

            for leg_id in LegId:
                leg_phase = step_phase + leg_relative_phase[leg_id.value]

                if leg_phase < 0:
                    leg_phase = 1.0 + leg_phase
                assert 0.0 <= leg_phase <= 1.0
                log.debug('leg = {0}, leg_phase = {1}, time = {2}'.
                          format(leg_id.value, leg_phase, tick_start))

                # Is this leg in ground contact?
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
                    #Lp[leg_id.value, :2] = [foot_x + Lp_neutral[leg_id.value, 0], foot_z + Lp_neutral[leg_id.value, 1]]
                    #Lp[leg_id.value, :2] = [foot_x                              , foot_z + Lp_neutral[leg_id.value, 1]]
                    # FIXME uing the constant below is a HACK.  Need to compute it
                    x_neutral = Lp_neutral[leg_id.value, 0]
                    #z_neutral = Lp_neutral[leg_id.value, 1]
                    Lp[leg_id.value, :2] = [foot_x + x_neutral, foot_z - body_center_up]
                    # DEBUGGING ONLY
                    # limit_check_lp(Lp, check_id='air')

                else:
                    ### This foot is in contact with the ground. ###
                    num_feet_on_ground += 1

                    # We know this foot is stationary relative to the ground
                    # and moves relative to the robot exactly opposite
                    # to the robot's velocity vector. So we move the foot
                    # by the -1 times amount the robot moves in one loop period
                    # fixme this should be a vector calculation
                    ##Lp[leg_id.value, 0:3] -= velocity_vector * loop_period
                    Lp[leg_id.value, 0] -= velocity_vector[0] * loop_period
                    Lp[leg_id.value, 1]   = -body_center_up
                    # DEBUGGING ONLY
                    # limit_check_lp(Lp, check_id='ground')
        else:
            # The robot is "standing", that is not walking and velocity is zero
            num_feet_on_ground = 4      # When AT_REST we assume 4 feet on ground
            step_start = 0.             # Step phase does not advance when AT_REST
            step_phase = 0.

        # Check if all four feet are on the ground, if so then we can
        # update any of the step parameters
        if num_feet_on_ground == 4:

            # Now is the time, with all four feet down
            # to adjust the step parameters.
            if velocity != last_velocity:

                # We treat zero velocity as a special case.
                if velocity == 0.0:
                    step_length = 0.0
                    step_period = 0.0
                    last_velocity = 0.0
                    standing = True
                else:
                    step_length, step_period = get_length_period(velocity)
                    last_velocity = velocity
                    standing = False

                # Velocity should be a vector, not a scaler as this robot can
                # walk in a direction it is not facing.
                velocity_vector = (velocity, 0.0, 0.0)  # (forward, up, left)

        # fixme -- add in any body position and angle needed for things like balance
        #roll_correction, pitch_correction = balance.rotate_to_level(imu.get_acceleration())
        roll_correction, pitch_correction = 0.0, 0.0
        
        # Find the joint angles to move the foot to the required position
        body_angles = (roll_correction, pitch_correction, 0.0)
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
    log.debug('walk_thread hits return')
    return

class Robot:

    def __init__(self):
        self.robot_state = 'inactive'
        return

    def kill(self):
        global walk_parameter_lock
        global run_walking_control_loop

        log.debug('kill entered at {0}'.format(time.time()))

        with walk_parameter_lock:
            run_walking_control_loop = False

        self.robot_state = 'inactive'
        return

    def stop_natural(self):
        """ DO NOT USE THIS, I am going to remove it """
        log.warning('Need to remove call to Stop_Natural()')
        self.set_velocity(0.0)


    def stop_panic(self):
        """NO NOT USE THIS"""
        log.warning('Need to remove call to Stop_panic()')
        self.kill()
        return

    def set_velocity(self, velocity: float):
        """Sets global velocity variable"""
        global walk_parameter_lock
        global gbl_velocity

        with walk_parameter_lock:
            gbl_velocity = velocity
        return

    def walk(self, velocity: float):
        """ The robot walks"""

        global run_walking_control_loop
        global gbl_velocity
        global walk_parameter_lock

        log.debug('walk entered at {0}'.format(time.time()))

        with walk_parameter_lock:
            run_walking_control_loop = True
            gbl_velocity = velocity

        # Create and start a walking thread.
        if not self.robot_state == 'walking':
            self.walk_thread = threading.Thread(target=walking, args=(), daemon=True)
            log.debug("starting walk_thread")
            self.robot_state = 'walking'
            self.walk_thread.start()
        else:
            log.debug("walk_thread already active")

        log.debug("walk returning")
        return

    def set_body_angles(self, angles: (float, float, float)):
        pass

    def set_body_center(self, body_point: (float, float, float)):

        global walk_parameter_lock

        global gbl_body_center_up
        global gbl_body_center_left
        global gbl_body_center_forward

        forward = body_point[0]
        assert -0.100 <= forward <= 0.100

        left    = body_point[1]
        assert -0.100 <= left    <= 0.100

        up      = body_point[2]
        assert  0.120 <= up      <= 0.220

        log.debug('acquiring the lock...')
        if walk_parameter_lock.acquire(blocking=True, timeout=0.5):

            gbl_body_center_forward = forward
            gbl_body_center_left    = left
            gbl_body_center_up      = up

            walk_parameter_lock.release()
        else:
            log.error('failed to acquire lock')

    def set_stance_width(self, stance_width: float):
        pass

    def set_stance_length(self, stance_length: float):
        pass

def walk_test(duration):
    assert duration >   0.0
    assert duration <= 10.0

    r = Robot()
    r.walk(0.05)
    time.sleep(duration)
    r.stop_natural()
    r.kill()
    return



if __name__ == "__main__":

    logging.basicConfig(filename='quad_controller.log',
                        filemode='w',
                        level=logging.DEBUG, )

    walk_test()
""" Defines the robot class and some actions the robot can perform and the main real-time control loop.
"""
import time
import threading
import logging
import math
import numpy as np
import sm_kinematics

log = logging.getLogger(__name__)


from enum import Enum

import config
# import joints
import pca9685_psd
import balance
import footpath


# FIXME
# import imu
# imu = imu.IMU()

# We always count and order the legs this way, left to right and front to rear
class LegId(Enum):
    LF = 0  # Left Front is first and number 0
    RF = 1
    LR = 2
    RR = 3  # Right Rear is last and number 3


# These are the things the robot can do.  So the robot can be in the
# "walk state" or the "neutral_stand state" or one of the others
class BehaviorState(Enum):
    INITIAL_POWERUP = 0     # Legs are in unknown position
    NEUTRAL_STAND   = 1     # Standing straight at medium height and weight equal on all feet
    WALK            = 2     # Actively walking, although velocity could be zero
    SIT             = 3     # Sit posture, with body tilted forward end upward


# These are sub-states. Each of the above states can have one of these sub-states.
class BehaviorSubState(Enum):
    STARTING     = 0       # Transition to current Behavior State
    CONTINUING   = 1
    TERMINATING  = 2       # Behavior is ending


init_time = time.time()
prior_state = BehaviorState.INITIAL_POWERUP
robot_substate = BehaviorSubState.STARTING

# PARAMETERS ##########################
neutral_body_height = 0.200
neutral_stance_width = 0.160
neutral_stance_length = 0.265


# Inter Task GLOBAL VARIABLES ####################
# These are used for inter-task communication.  The real-time control task
# reads them and the GUI or other controller task sets them.
# Always use the lock to access these.
gbl_run_control_loop = False

gbl_robot_state    = BehaviorState.INITIAL_POWERUP

# The next three variables are equivalent to the ROS "twist" message.
gbl_velocity_x = 0.0
gbl_velocity_y = 0.0
gbl_rot_rate_z = 0.0

gbl_stance_width   = neutral_stance_width
gbl_stance_length  = neutral_stance_length
gbl_body_center_up = neutral_body_height

gbl_body_center_left    = 0.0
gbl_body_center_forward = 0.0

gbl_body_angle_roll  = 0.0
gbl_body_angle_pitch = 0.0
gbl_body_angle_yaw   = 0.0

# Time to move from previous state to a new static pose such as "sit" or "stand"
# This should be set to something reasonable before changing the state to one of the static pose states
gbl_static_transition_time = 0.0

gbl_lock = threading.Lock()
## END GLOBALS #################



def limit_check_lp(Lp, check_id=''):
    """Perform a diagnostic error check on Lp array -- remove this in production code"""
    if abs(np.amax(Lp[:, :3])) > 0.300:
        # ERROR something is wrong, the legs are not over 300 mm long
        log.error('Lp array limit check failed')
        print(check_id, Lp)
    return


def neutral_stance_Lp() -> np.array:
    """Function to return a reasonable neutral stance on flat ground"""

    global gbl_stance_width
    global gbl_body_center_up
    global gbl_run_control_loop

    # Units for these are Meters.  Change the constants as opion about
    # what is best evolves.
    bias = -0.065

    if gbl_lock.acquire(blocking=True, timeout=0.5):
        stance_width  = gbl_stance_width
        stance_length = gbl_stance_length
        vertical      = gbl_body_center_up
        gbl_lock.release()
    else:
        log.warning('neutral_stance_Lp() failed to acquire lock')

    stance_halfwidth  = stance_width  / 2.0
    stance_halflength = stance_length / 2.0

    # Distances of each foot from body center
    #                   fore/aft               height      left/right
    Lp = np.array([[stance_halflength + bias, -vertical, stance_halfwidth, 1],  # LF
                   [stance_halflength + bias, -vertical, -stance_halfwidth, 1],  # RF
                   [-stance_halflength + bias, -vertical, stance_halfwidth, 1],  # LR
                   [-stance_halflength + bias, -vertical, -stance_halfwidth, 1]])  # RR

    limit_check_lp(Lp, check_id='get_neural_stance_Lp')
    return Lp


tick_start = 0.0
lp_neutral = neutral_stance_Lp()
lp = lp_neutral.copy()

velocity_x = 0.0
velocity_y = 0.0
rot_rate_z = 0.0
fp = footpath.Footpath()
body_center_up = 0.
transition_time = 0.


def control_loop():
    """
    Realtime control loop for robot base controller.

    This function should be called as a task.  After some setup, a loop is
    entered that runs many times per second.  The loop will,
      1) read shared variables, after obtaining lock
      2) compute the desired location in (x,y,z), body relative for each foot
      3) perform inverse kinetics to find the joint angles for each leg
      4) update the servo motors to move the joints to the desired angle
    The loop continues to run until gbl_run_control_loop is set to False.

    Returns: none.

    """

    ### GLOBALS used for inter task communication
    global gbl_lock
    global gbl_run_control_loop

    global gbl_robot_state

    global gbl_body_center_up
    global gbl_body_center_left
    global gbl_body_center_forward

    global gbl_body_angle_roll
    global gbl_body_angle_pitch
    global gbl_body_angle_yaw

    global prior_state
    global robot_substate

    global velocity_x
    global velocity_y
    global rot_rate_z
    global tick_start
    global body_center_up

    global gbl_static_transition_time
    global transition_time

    ### Channel number on PCA9685 for each angle
    servo_channel = np.array(config.servo_channel_assignment,
                             dtype=np.uint8)
    # Class to abstract real and simulated servo hardware
    # servos = joints.ServoShim()
    s = pca9685_psd.Servo()

    # smk (Spot Micro Kinematic) object
    smk = sm_kinematics.Kinematic()

    # Heartbeat will blink a LED.
    heartbeat_time = 0.0

    loop_period = 1.0 / config.rt_loop_freq
    run_control_loop = True  # Set True in case acquire lock fails.
    while True:
        tick_start = time.time()

        # copy global parameters
        if gbl_lock.acquire(blocking=True, timeout=loop_period / 2.0):
            run_control_loop = gbl_run_control_loop

            robot_state = gbl_robot_state

            velocity_x = gbl_velocity_x
            velocity_y = gbl_velocity_y
            rot_rate_z = gbl_rot_rate_z

            body_center_up      = gbl_body_center_up
            body_center_left    = gbl_body_center_left
            body_center_forward = gbl_body_center_forward

            body_angle_roll  = gbl_body_angle_roll
            body_angle_pitch = gbl_body_angle_pitch
            body_angle_yaw   = gbl_body_angle_yaw

            transition_time = gbl_static_transition_time

            gbl_lock.release()
        else:
            log.warning('control_loop() failed to acquire lock {0}'.format(tick_start))
            print('control_loop() failed to acquire lock {0}'.format(tick_start)) #TODO

        # Check if this thread should continue running
        if not run_control_loop:
            log.info('control loop terminating at {0}'.format(tick_start))
            print('control loop terminating at {0}'.format(tick_start)) #TODO
            break

        # Heartbeat -- show the world this thread is alive
        if tick_start > heartbeat_time:
            #FIXME need to blink a real LED
            log.debug('heartbeat at {0}'.format(tick_start))
            print('heartbeat at {0}'.format(tick_start)) #TODO remove
            heartbeat_time = tick_start + config.rt_heartbeat_period

        # Did the robot state just change?  If so then the sub state has to be "STARTING"
        if robot_state != prior_state:
            robot_substate = BehaviorSubState.STARTING
            prior_state = robot_state

        if robot_state == BehaviorState.INITIAL_POWERUP:
            # In this state, there is nothing to do.
            continue

        elif robot_state == BehaviorState.NEUTRAL_STAND:
            control_stand()
        elif robot_state == BehaviorState.WALK:
            control_walk()
        elif robot_state == BehaviorState.SIT:
            control_sit()
        else:
            log.error('Invalid State, resetting')
            robot_state = BehaviorState.INITIAL_POWERUP

        #FIXME -- add in any body position and angle needed for things like balance
        # roll_correction, pitch_correction = balance.rotate_to_level(imu.get_acceleration())
        roll_correction, pitch_correction = 0.0, 0.0

        # Find the joint angles to move the foot to the required position
        body_angles = (body_angle_roll + roll_correction,
                       body_angle_yaw,
                       body_angle_pitch + pitch_correction)
        body_center = (body_center_forward,
                       body_center_up-neutral_body_height,
                       body_center_left)

        joint_angles = smk.calcIK(lp, body_angles, body_center)

        # Set the robots joints to the angles computed above.
        # Note that the function below will check if the motors are enabled or if
        # we are running in simulation only.  So maybe the angles only go to a log
        # file, or they could move the physical robot.
        """
        for leg_idx, leg in enumerate(joint_angles):
            for theta_idx, theta in enumerate(leg):
                servos.set_angle(servo_channel[leg_idx, theta_idx], theta)"""

        # Turn (4, 3) array into a (16,) array.
        # First make it a (4, 4) and then flatten it
        joint_angles_flattened = np.append(joint_angles, [[0.0], [0.0], [0.0], [0.0]], 1).reshape(-1)
        s.move_16_radian(joint_angles_flattened)

        tick_elapsed = time.time() - tick_start
        sleep_time = loop_period - tick_elapsed

        if sleep_time < 0.001:
            log.warning('loop overrun, elapsed = {0}'.format(tick_elapsed))
        else:
            assert 0.001 < sleep_time < loop_period, 'invalid sleep_time {0}'.format(sleep_time)
            time.sleep(sleep_time)

    # this task ends
    log.debug('control loop hits return')
    return


stand_start_time = 0.0
lp_zero = np.array([[0., 0., 0., 0.],
                    [0., 0., 0., 0.],
                    [0., 0., 0., 0.],
                    [0., 0., 0., 0.]])

# Table specifies the phase when each foot begins to move and when it finishes
# phase the fraction of the transition_time that has past since stand_start_time.
# FIXME This table needs to be computed based on distance each foot is to be moved.
#
#             Start  End
leg_phase = [(0.00, 0.20),  # leg 0 LF
             (0.50, 0.70),  # leg 1 RF
             (0.25, 0.45),  # leg 2 LR
             (0.75, 0.95)]  # leg 4 RR


def control_stand() -> None:

    global gbl_lock


    global robot_substate
    global stand_start_time
    global lp_zero
    global lp_neutral

    global transition_time
    global lp

    if robot_substate == BehaviorSubState.STARTING:

        # Substate is "starting" so we run the initialization code then set state to
        # "continueing"
        stand_start_time = time.time()

        lp_zero = lp.copy()   # save current feet location
        lp_neutral = neutral_stance_Lp()
        robot_substate = BehaviorSubState.CONTINUING

    if robot_substate == BehaviorSubState.CONTINUING:

        phase = (time.time() - stand_start_time) / transition_time
        if phase < 1.0:
            lp =  (phase * lp_zero) +  ((1.0 - phase) * lp_neutral)

        else:
            lp = lp_neutral.copy()

    else:
        log.error('Invalid Substate')
    return


step_start = 0.0
#                     LF     RF     LR     RR
leg_relative_phase = [0.00, -0.50, -0.50, -0.00]
static_step_air_fraction = 0.0
last_velocity = (0., 0., 0.)
step_period = 0.001 # 0.001 is a hack to avid potential for divide by zero error


def control_walk():

    global gbl_lock
    global robot_substate
    global step_start
    global tick_start
    global leg_relative_phase
    global static_step_air_fraction
    global body_center_up
    global last_velocity
    global step_period

    if robot_substate == BehaviorSubState.STARTING:
        step_start = tick_start
        robot_substate = BehaviorSubState.CONTINUING

    if robot_substate == BehaviorSubState.CONTINUING:

        # if the robot is not moving, there is nothing to do...
        if (abs(velocity_x) + abs(velocity_y)) < 0.002:
            return

        # Only call fp.set_velocity on change
        if last_velocity != (velocity_x, velocity_y, rot_rate_z):
            step_period = fp.set_velocity(velocity_x, velocity_y, rot_rate_z)
            last_velocity = (velocity_x, velocity_y, rot_rate_z)

        # get the current step phase, handle wrap-around.  Phase can never
        # be greater than 1.0
        assert step_period > 0.01  # verify a sane value
        step_phase = (tick_start - step_start) / step_period
        if step_phase > 1.0:
            (step_phase, _) = math.modf(step_phase)
            step_start = tick_start - (step_period * step_phase)

        assert 0.0 <= step_phase <= 1.0, \
            'invalid Step Phase {0}, {1}, {2}, {3}'. \
                format(step_phase, tick_start, step_start, step_period)

        num_feet_on_ground = 0
        for leg_id in LegId:
            leg_abs_phase = step_phase + leg_relative_phase[leg_id.value]

            if leg_abs_phase < 0:
                leg_abs_phase = 1.0 + leg_abs_phase
            assert 0.0 <= leg_abs_phase <= 1.0

            fx, fy, fz = fp.xyz(leg_abs_phase)
            x_neutral = lp_neutral[leg_id.value, 0]
            y_neutral = lp_neutral[leg_id.value, 2]
            lp[leg_id.value, :3] = [fx + x_neutral,
                                    fz - body_center_up,
                                    fy + y_neutral]
            if fz == 0.0:
                num_feet_on_ground += 1

        # Check if all four feet are on the ground, if so then we can
        # update any of the step parameters
        if num_feet_on_ground == 4:
            # Now is the time, with all four feet down
            # to adjust the step parameters.
            step_period = fp.set_velocity(velocity_x, velocity_y, rot_rate_z)


def control_sit():
    pass


class Robot:
    """
    This class provides basic controls for the robot base controller.

    The Robot class provides a set of functions that return immediately, that are
    use to control the robot's basic motion and behaviors such as walk, sit, or
    stand.  Creating a Robot object will start a task initialize the real-time
    control loop.  The Robot class then has methods to create movements and to
    get status.
    """

    def __init__(self):
        """Create a Robot object and a task to control it."""

        self.lock_timeout = 0.5
        self.__start_control_loop_thread()

        return

    def kill(self):
        """Quickly stops the realtime control task."""

        global gbl_lock
        global gbl_run_control_loop
        global gbl_robot_state

        if gbl_lock.acquire(blocking=True,
                            timeout=self.lock_timeout):
            gbl_run_control_loop = False
            gbl_robot_state = BehaviorState.INITIAL_POWERUP
            gbl_lock.release()
        else:
            log.error('kill() failed to acquire lock')
        return


    def set_velocity(self, vel_x: float = 0.0, vel_y: float = 0.0, rot_z: float = 0.0):
        """Sets the robot's velocity and rate of turn over ground."""

        global gbl_lock
        global gbl_velocity_x
        global gbl_velocity_y
        global gbl_rot_rate_z

        if gbl_lock.acquire(blocking=True,
                            timeout=self.lock_timeout):
            gbl_velocity_x = vel_x
            gbl_velocity_y = vel_y
            gbl_rot_rate_z = rot_z
            gbl_lock.release()
        else:
            log.error('set_velocity() failed to acquire lock')
        return

    def walk(self, vel_x: float = 0.0, vel_y: float = 0.0, rot_z: float = 0.0):
        """Cause robot to walk."""

        global gbl_lock
        global gbl_robot_state

        self.set_velocity(vel_x, vel_y, rot_z)

        if gbl_lock.acquire(blocking=True,
                            timeout=self.lock_timeout):
            gbl_robot_state = BehaviorState.WALK
            gbl_lock.release()
        else:
            log.error('walk() failed to acquire lock')
        return

    def stand(self, transition: float = 1.0):
        """Cause robot to stand in a neutral position."""

        global gbl_lock
        global gbl_robot_state
        global gbl_static_transition_time

        if gbl_lock.acquire(blocking=True,
                            timeout=self.lock_timeout):
            gbl_static_transition_time = transition
            gbl_robot_state    = BehaviorState.NEUTRAL_STAND
            gbl_lock.release()
        else:
            log.error('stand() failed to acquire lock')
        return

    def sit(self, transition_time: float = 0.0):
        """Cause robot to sit."""

        global gbl_lock
        global gbl_robot_state

        if gbl_lock.acquire(blocking=True,
                            timeout=self.lock_timeout):
            gbl_robot_state    = BehaviorState.SIT
            gbl_lock.release()
        else:
            log.error('failed to acquire lock')
        return

    def set_body_angles(self, body_angle: (float, float, float) = (0.0, 0.0, 0.0)):
        """Set body rotations with respect to neutral position."""

        global gbl_lock

        global gbl_body_angle_roll
        global gbl_body_angle_pitch
        global gbl_body_angle_yaw

        limit = math.radians(21.0)  # limit for all angles

        roll = body_angle[0]
        assert -limit <= roll <= limit, f"roll = {roll}"

        pitch = body_angle[1]
        assert -limit <= pitch <= limit, f"pitch = {pitch}"

        yaw = body_angle[2]
        assert -limit <= yaw <= limit, f"yaw = {yaw}"

        if gbl_lock.acquire(blocking=True,
                            timeout=self.lock_timeout):

            gbl_body_angle_roll = roll
            gbl_body_angle_pitch = pitch
            gbl_body_angle_yaw = yaw

            gbl_lock.release()
        else:
            log.error('failed to acquire lock')

    def set_body_center(self, body_point: (float, float, float)):
        """Set body translations with respect to neutral position."""

        global gbl_lock

        global gbl_body_center_up
        global gbl_body_center_left
        global gbl_body_center_forward

        forward = body_point[0]
        assert -0.100 <= forward <= 0.100

        left = body_point[1]
        assert -0.100 <= left <= 0.100

        up = body_point[2]
        assert 0.120 <= up <= 0.240

        log.debug('acquiring the lock...')
        if gbl_lock.acquire(blocking=True,
                            timeout=self.lock_timeout):

            gbl_body_center_forward = forward
            gbl_body_center_left = left
            gbl_body_center_up = up

            gbl_lock.release()
        else:
            log.error('failed to acquire lock')

    def set_stance_width(self, stance_width_delta: float):
        """Set the distance between the left and right feet

        Parms:
            stance_width_delta  Reduction from nominal stance in meters
        """



        global gbl_lock
        global gbl_stance_width

        assert -0.050 < stance_width_delta < 0.050

        if gbl_lock.acquire(blocking=True,
                            timeout=self.lock_timeout):
            gbl_stance_width = neutral_stance_width + stance_width_delta
            gbl_lock.release()
        else:
            log.error('failed to acquire lock')
        return

    def set_stance_length(self, stance_length: float):
        """Set the distance between the front and rear feet."""

        global gbl_lock
        global gbl_stance_length

        if gbl_lock.acquire(blocking=True,
                            timeout=self.lock_timeout):
            gbl_stance_length = stance_length
            gbl_lock.release()
        else:
            log.error('failed to acquire lock')
        return

    def __start_control_loop_thread(self):
        """Private method to setup and run the real time control loop."""
        global gbl_lock
        global gbl_run_control_loop
        global gbl_robot_state
        global gbl_robot_substate


        if gbl_lock.acquire(blocking=True,
                            timeout=self.lock_timeout):
            loop_was_running = gbl_run_control_loop

            gbl_run_control_loop = True
            gbl_robot_state = BehaviorState.INITIAL_POWERUP
            gbl_robot_substate = BehaviorSubState.STARTING

            gbl_lock.release()
        else:
            log.error('failed to acquire lock')

        if loop_was_running:
            log.warning("control loop thread already active")
            print("control loop thread already active") #TODO remove
        else:

            self.control_loop_thread = threading.Thread(target=control_loop, args=(), daemon=True)
            log.debug("starting control_loop_thread")
            print("starting control_loop_thread") #TODO remove
            self.control_loop_thread.start()

        log.debug("control loop thread returning")
        return


def __walk_test(duration):
    """Debugging test, cause the robot to walk for a few seconds."""

    assert duration > 0.0
    assert duration <= 20.0

    r = Robot()
    r.walk(0.05, 0, 0)
    time.sleep(duration)
    r.set_velocity(0.0, 0.0, 0.0)
    r.stand()
    r.kill()
    return


if __name__ == "__main__":
    logging.basicConfig(filename='quad_controller.log',
                        filemode='w',
                        level=logging.DEBUG)
    __walk_test(10)

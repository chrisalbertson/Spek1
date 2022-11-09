""" Top level class that defines the robot object and some actions the robot can perform
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
import joints
import balance
import footpath

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

## PARAMETERS ##########################
neutral_body_height  = 0.200
neutral_stance_width = 0.160

## GLOBAL VARIABLES ####################
gbl_velocity   = 0.0
gbl_velocity_y = 0.0
gbl_turnrate   = 0.0
run_walking_control_loop = False

gbl_stance_width = 100.0

gbl_body_center_up      = neutral_body_height
gbl_body_center_left    = 0.0
gbl_body_center_forward = 0.0

gbl_body_angle_roll  = 0.0
gbl_body_angle_pitch = 0.0
gbl_body_angle_yaw   = 0.0

walk_parameter_lock = threading.Lock()
## END GLOBAL VERIABLES #################


def get_delta_y(air_fraction_completed,
                step_period,
                step_length,
                rotation_rate) -> float:
                
    #FIXME, we should cache "y_per_step" as it only needs to be computed
    # once per step.            
    angle_per_step = rotation_rate * step_period
    y_per_step = step_length * math.sin(angle_per_step) 
    return air_fraction_completed * y_per_step               
                
                
                
                
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
    

    # Units for these are Meters.  Change the constants as opion about
    # what is best evolves.
    vertical      =  neutral_body_height 
    stance_width  =  neutral_stance_width
    stance_length =  0.260
    bias          = -0.065
    
    with walk_parameter_lock:
        gbl_stance_width   = stance_width
        gbl_body_center_up = vertical

    stance_halfwidth  = stance_width  / 2.0
    stance_halflength = stance_length / 2.0


    # Distances of each foot from body center
    #                   fore/aft               height      left/right
    Lp = np.array([[ stance_halflength + bias, -vertical,  stance_halfwidth, 1],   # LF
                   [ stance_halflength + bias, -vertical, -stance_halfwidth, 1],   # RF
                   [-stance_halflength + bias, -vertical,  stance_halfwidth, 1],   # LR
                   [-stance_halflength + bias, -vertical, -stance_halfwidth, 1]])  # RR

    limit_check_lp(Lp, check_id='get_neural_stance_Lp')
    return Lp


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


def walking():

    global run_walking_control_loop
    global walk_parameter_lock

    global gbl_body_center_up
    global gbl_body_center_left
    global gbl_body_center_forward
    
    global gbl_body_angle_roll
    global gbl_body_angle_pitch
    global gbl_body_angle_yaw

    fp = footpath.Footpath()

    ### Channel number on PCA9685 for each angle
    #                         th1 th2 th3
    servo_channel = np.array([[ 0,  1,  2],    # LF
                              [ 4,  5,  6],    # RF
                              [ 8,  9, 10],    # LR
                              [12, 13, 14]],   # RR
                             dtype=np.uint8)
    servos = joints.ServoShim()
    smk = sm_kinematics.Kinematic()

    # numpy array to hold position of all four feet
    Lp_neutral = neutral_stance_Lp()
    Lp = Lp_neutral.copy()
    limit_check_lp(Lp, check_id='stance')

    # Set the leg phases and "air fraction" to define an amble gait
    # where one legs moves forward at a time
    # In this gait, leg LF leads and the others follow

    leg_relative_phase = [0., 0., 0., 0.]
    gait = 'trot_faststep'

    ## GAIT DEFINITIONS ################
    ##
    ## NOTES:
    ##   1. if step_air_fracton == 0.0 this is a flag to indicate it
    ##      should be set dynamically.
    
    if gait == 'trot_faststep':
        static_step_air_fraction = 0.0
        leg_relative_phase[LegId.LF.value] =  0.00
        leg_relative_phase[LegId.RR.value] =  0.00
        leg_relative_phase[LegId.RF.value] = -0.50
        leg_relative_phase[LegId.LR.value] = -0.50
    elif gait == 'creep':
        static_step_air_fraction = 0.08
        leg_relative_phase[LegId.LF.value] =  0.00
        leg_relative_phase[LegId.RR.value] = -0.25
        leg_relative_phase[LegId.RF.value] = -0.50
        leg_relative_phase[LegId.LR.value] = -0.75
    else:
        log.error('invalid gait')

    # Get Velocity, so we can compute initial step length and period
    with walk_parameter_lock:
        velocity = gbl_velocity
        velocity_y = gbl_velocity_y
        turnrate = gbl_turnrate

        body_center_up      = gbl_body_center_up
        body_center_left    = gbl_body_center_left
        body_center_forward = gbl_body_center_forward
         
        body_angle_roll  = gbl_body_angle_roll
        body_angle_pitch = gbl_body_angle_pitch
        body_angle_yaw   = gbl_body_angle_yaw

    last_velocity = velocity
    step_period = fp.set_velocity(velocity, velocity_y, turnrate)

    heartbeat_time = 0.0
    heartbeat_period = 1.0

    loop_frequency = 50.0  #FIXME this should come from config
    loop_period = 1.0 / loop_frequency
    step_start = 0.

    while True:
        tick_start = time.time()

        # copy global parameters
        log.debug('acquiring the lock...')
        if walk_parameter_lock.acquire(blocking=True, timeout=0.010):
            run = run_walking_control_loop
            velocity   = gbl_velocity
            velocity_y = gbl_velocity_y
            turnrate   = gbl_turnrate
            
            body_center_up =      gbl_body_center_up
            body_center_left =    gbl_body_center_left
            body_center_forward = gbl_body_center_forward
         
            body_angle_roll  = gbl_body_angle_roll
            body_angle_pitch = gbl_body_angle_pitch
            body_angle_yaw   = gbl_body_angle_yaw
     
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

        ####### GOTO NEUTRAL STANCE ########

        # How far must each foot move?
        """
        dist_to_neutral = [0, 0, 0, 0]
        dist_total = 0.0
        for leg in LegId:
            current_pnt = Lp[leg.value, :3]
            neutral_pnt = Lp_neutral[leg.value, :3]
            d = math.dist(current_pnt, netral_pnt)
            dist_to_neutral[leg.value] = d
            dist_total += d
            """

        ####### END GOTO NEUTRAL STANCE ########

        ####### WALKING #######

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

        assert 0.0 <= step_phase <= 1.0, \
                'invalid Step Phase {0}, {1}, {2}, {3}'.\
                format(step_phase, tick_start, step_start, step_period)

        num_feet_on_ground = 0
        for leg_id in LegId:
            leg_phase = step_phase + leg_relative_phase[leg_id.value]

            if leg_phase < 0:
                leg_phase = 1.0 + leg_phase
            assert 0.0 <= leg_phase <= 1.0

            fx,fy,fz = fp.xyz(leg_phase)
            x_neutral = Lp_neutral[leg_id.value, 0]
            y_neutral = Lp_neutral[leg_id.value, 2]
            Lp[leg_id.value, :3] = [fx + x_neutral,
                                    fz - body_center_up,
                                    fy + y_neutral]
            if fz == 0.0:
                num_feet_on_ground += 1

        # Check if all four feet are on the ground, if so then we can
        # update any of the step parameters
        if num_feet_on_ground == 4:

            # Now is the time, with all four feet down
            # to adjust the step parameters.
            step_period = fp.set_velocity(velocity, velocity_y, turnrate)

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

        joint_angles = smk.calcIK(Lp, body_angles, body_center)

        ######## WALKING END ########

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
            assert 0.001 < sleep_time < loop_period, 'invalid sleep_time {0}'.format(sleep_time)
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
        log.error('Need to remove call to Stop_Natural()')
        self.set_velocity(0.0)


    def stop_panic(self):
        """NO NOT USE THIS"""
        log.warning('Need to remove call to Stop_panic()')
        self.kill()
        return

    def set_velocity(self, velocity: float, vel_y: float = 0.0):
        """Sets global velocity variable"""
        global walk_parameter_lock
        global gbl_velocity
        global gbl_velocity_y

        with walk_parameter_lock:
            gbl_velocity = velocity
            gbl_velocity_y = vel_y
        return

    def set_turnrate(self, turnrate: float):
        """Sets global turnrate variable"""
        global walk_parameter_lock
        global gbl_turnrate

        with walk_parameter_lock:
            gbl_turnrate = turnrate
        return

    def walk(self, velocity: float, vel_y: float = 0.0):
        """ The robot walks"""

        global run_walking_control_loop
        global gbl_velocity
        global gbl_velocity_y
        global walk_parameter_lock

        log.debug('walk entered at {0}'.format(time.time()))

        with walk_parameter_lock:
            run_walking_control_loop = True
            gbl_velocity   = velocity
            gbl_velocity_y = vel_y

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

    def set_body_angles(self, body_angle: (float, float, float)):

        global walk_parameter_lock

        global gbl_body_angle_roll
        global gbl_body_angle_pitch
        global gbl_body_angle_yaw
        
        limit = math.radians(21.0) #limit for all angles

        roll = body_angle[0]
        assert -limit <= roll <= limit,  f"roll = {roll}"

        pitch    = body_angle[1]
        assert -limit <= pitch <= limit, f"pitch = {pitch}"

        yaw      = body_angle[2]
        assert -limit <= yaw <= limit,   f"yaw = {yaw}"

        log.debug('acquiring the lock...')
        if walk_parameter_lock.acquire(blocking=True, timeout=0.5):

            gbl_body_angle_roll   = roll
            gbl_body_angle_pitch  = pitch
            gbl_body_angle_yaw    = yaw

            walk_parameter_lock.release()
        else:
            log.error('failed to acquire lock')

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
        assert  0.120 <= up      <= 0.240

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

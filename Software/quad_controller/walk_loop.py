
import time
import math
import threading

import logging
log = logging.getLogger(__name__)

import config
import global_state as gs
import leg
import servo_shim as ss


"""This is the real-time control look that computes and sets every robot joint

The loop runs at somewhere between 10 and 100 Hz and controls the body and leg
positions and orientation.
"""
def control_loop():

    log.info('control_loop starting')

    # TBD for now lets just do only one leg.
    # That is enough to show we can do more than one.
    legLR = leg.Leg()
    
    Servo = ss.servo_shim()


    next_heartbeat = 0.0

    loop_frequency = 20.0  # Hz.
    loop_period = 1.0 / loop_frequency
    walk_start_time = time.time()
    while 1:
        tick_start = time.time()

        # grab globals
        gs.GlobalLock.acquire()
        step_period = gs.StepPeriod
        step_length = gs.StepLength
        step_height = gs.StepHeight
        gait = gs.Gait
        gs.GlobalLock.release()

        # Is it time to heartbeat yet?
        if tick_start > next_heartbeat:
            next_heartbeat = tick_start + 1.0
            # TBD - blink a LED
            logging.debug('Heartbeat at', tick_start, step_period, step_length, step_height, gait)

        #Get the current step phase
        # FIXME this will not work if the step period is changed.
        step_period = 2.0  # fixme
        step_time = (tick_start - walk_start_time) % step_period
        step_phase = step_time / step_period

        # find the Y,Z location for the foot
        ground_contact, (footY, footZ) = legLR.get_legYZ(step_phase)
       

        # Find knee and hip angle using inverse kinematics
        # FIXME trivial ground to hip frame translation, assume fixed body height
        hip_to_footY = footY
        hip_to_footZ = 0.120 - footZ
        angle_hip, angle_knee = legLR.ik2d( (hip_to_footY, hip_to_footZ))

        # Send the angle to the servo motors
        Servo.set_angle(13, angle_hip)
        Servo.set_angle(14, angle_knee)

        # Wait a little less than loop_period
        tick_elapsed = time.time() - tick_start
        sleep_time = loop_period - tick_elapsed
        if sleep_time < 0.001:
            log.warning('control loop over run, loop elapsed time', tick_elapsed)
        else:
            time.sleep(sleep_time)



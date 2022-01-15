
import time
import math
import threading

import logging
log = logging.getLogger(__name__)

import config

# we can use one of these
if config.ui_web_gui:
    import PySimpleGUIWeb as sg
elif config.ui_x11_gui:
    import PySimpleGUI as sg

#import global_state as gs
import leg
import servo_shim as ss

######################################
# GLOBALS
run_control_loop = True
# end GLOBALS
######################################

"""This is the real-time control loop that computes and sets every robot joint

The loop runs at somewhere between 10 and 100 Hz and controls the body and leg
positions and orientation.
"""
def control_loop():

    log.info('control_loop starting')

    global run_control_loop

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

        if not run_control_loop:
            break


        # grab globals  FIXME  Get values from GUI
        #gs.GlobalLock.acquire()
        step_period = 1.000
        step_length = 0.050
        step_height = 0.010
        body_height = 0.120
        #gs.GlobalLock.release()

        # Is it time to heartbeat yet?
        if tick_start > next_heartbeat:
            next_heartbeat = tick_start + 1.0
            # TBD - blink a LED
            log.debug('Heartbeat')

        # Get the current step phase
        # Treat a period of 0.0 as a special case of "stopped" where the phase never
        # leaves zero
        if step_period > 0.05:
            step_time = (tick_start - walk_start_time) % step_period
            step_phase = step_time / step_period
        else:
            step_phase = 0.0

        # find the normalized Y,Z location for the foot then scale by current
        # step length and step height
        ground_contact, (footY, footZ) = legLR.get_legYZ(step_phase)
        footY = footY * step_length
        footZ = footZ * step_height

        # Find knee and hip angle using inverse kinematics
        # FIXME trivial ground to hip frame translation, assume fixed body height
        # and assume body translates in Y at the current walking speed
        hipZ = body_height
        hipY = step_phase * step_length
        hip_to_footY = footY - hipY
        hip_to_footZ = footZ - hipZ
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
    return


def run_gui():
    log.debug('run_gui...')

    global run_control_loop

    spaces = '      '
    walk_mode = 'slow'
    layout_sn = [[sg.Text('Choose parameters then press SET')],

                 [sg.HorizontalSeparator()],
                 [sg.Text(spaces), sg.Radio('slow', group_id=1, key='-S-')],
                 [sg.Text(spaces), sg.Radio('medium', group_id=1, key='-M-')],
                 [sg.HorizontalSeparator()],
                 [sg.Text(spaces), sg.Button('SET', key='-SET-')],
                 [sg.Text('Status:'), sg.Text(walk_mode, size=40, key='-STATUS-')],

                 [sg.HorizontalSeparator()],
                 [sg.Text(spaces), sg.Button('Exit')]
                 ]

    if config.ui_web_gui:
        window = sg.Window('Speck -- Walk', layout_sn, web_port=2222, web_start_browser=False)
    elif config.ui_x11_gui:
        window = sg.Window('Speck -- Walk', layout_sn)

    while True:  # Event Loop
        event, values = window.read()
        # print(event, values)

        if event in (None, 'Exit'):
            break

        if event == '-SET-':
            if values['-S-']:
                walk_mode = 'slow'
                # fixme - set parms for slow

            elif values['-M-']:
                walk_mode = 'medium'
                # fixme - set parms for medim


        # Hold the lock only long enouh to copy the global variable, no longer
        #GlobalLock.acquire()
        ## fixme - copy globals
        #GlobalLock.release()

        window['-STATUS-'].Update(walk_mode)

    #GlobalLock.acquire()
    run_control_loop = False
    #GlobalLock.release()

    window.close()
    return



def start():
    log.debug('start...')

    global run_control_loop

    run_control_loop = True
    walk_thread = threading.Thread(target=control_loop, args=(), daemon=True)
    walk_thread.start()

    run_gui()

    walk_thread.join()
    return

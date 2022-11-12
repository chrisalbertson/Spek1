""" This module implements the "stand activity" and also serves as a model for the "activity" design pattern"""

import time
import threading

import logging
log = logging.getLogger(__name__)

import config

# we can use one of these
if config.ui_web_gui:
    import PySimpleGUIWeb as sg
elif config.ui_x11_gui:
    import PySimpleGUI as sg

import leg
import servo_shim as ss


#####################################################
# GLOBALS
# These need to be LOCKED before accessing
run_stand_control_loop = False
stance_height = 'none'

# This is the lock that needs to be used
GlobalLock = threading.Lock()
# End GLOBALS
#####################################################


def control_loop():
    log.debug('control_loop...')

    global run_stand_control_loop
    global stance_height

    legs = leg.Leg()

    Servo = ss.servo_shim()

    loop_frequency = 5.0  # Hz.
    loop_period = 1.0 / loop_frequency
    while True:
        tick_start = time.time()

        GlobalLock.acquire()
        run = run_stand_control_loop
        height = stance_height
        GlobalLock.release()

        # This loop runs until the user interface thread sets run_stand_control_loop to False
        if not run:
            break

        x =  0.0
        y =  0.0

        if height == 'low':
            z = -0.100
        elif height == 'tall':
            z = -0.140
        else:
            z = -0.120

        # loop over all four legs
        for leg_index in range(4):
            th1, th2, th3 = legs.ik3d(x, y, z)

        Servo.set_angle(config.servo_channel[leg_index][0], th1)
        Servo.set_angle(config.servo_channel[leg_index][1], th2)
        Servo.set_angle(config.servo_channel[leg_index][2], th3)

        # Wait a little less than loop_period
        tick_elapsed = time.time() - tick_start
        sleep_time = loop_period - tick_elapsed
        if sleep_time > 0.001:
            time.sleep(sleep_time)
        else:
            # It took longer then the loop period to run the loop,
            # so we do not sleep.
            log.warning('control_loop over run, elapsed time', tick_elapsed)
    return


def run_gui():
    log.debug('run_gui...')

    global run_stand_control_loop
    global stance_height

    spaces = '    '
    stance_height = 'medium'
    layout_sn = [[sg.Text('Choose a stance and press MOVE')],

                 [sg.HorizontalSeparator()],
                 [sg.Text(spaces), sg.Radio('low', group_id=1, key='-L-')],
                 [sg.Text(spaces), sg.Radio('medium', group_id=1, key='-M-')],
                 [sg.Text(spaces), sg.Radio('tall', group_id=1, key='-T-')],
                 [sg.HorizontalSeparator()],
                 [sg.Text(spaces), sg.Button('MOVE', key='-MOVE-')],
                 [sg.Text('Status:'), sg.Text(stance_height, size=40, key='-STATUS-')],

                 [sg.HorizontalSeparator()],
                 [sg.Text(spaces), sg.Button('Exit')]
                 ]

    if config.ui_web_gui:
        window = sg.Window('Speck -- Stand', layout_sn, web_port=2222, web_start_browser=False)
    elif config.ui_x11_gui:
        window = sg.Window('Speck -- Stand', layout_sn)

    while True:  # Event Loop
        event, values = window.read()
        # print(event, values)

        if event in (None, 'Exit'):
            break

        if event == '-MOVE-':
            if values['-L-']:
                height = 'low'

            elif values['-M-']:
                height = 'medium'

            elif values['-T-']:
                height = 'tall'

        # Hold the lock only long enouh to copy the global variable, no longer
        GlobalLock.acquire()
        stance_height = height
        GlobalLock.release()

        window['-STATUS-'].Update(height)

    GlobalLock.acquire()
    run_stand_control_loop = False
    GlobalLock.release()

    window.close()
    return


def start():
    log.debug('start...')

    global run_stand_control_loop
    global stance_height

    run_stand_control_loop = True
    stand_thread = threading.Thread(target=control_loop, args=(), daemon=True)
    stand_thread.start()

    run_gui()

    stand_thread.join()
    return


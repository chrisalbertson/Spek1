
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

import test_xyz_gui as xyz
import joints
import robot


def manage_level1_webx11_gui():

    waiting = 'none -- waiting for input'
    spaces = '    '

    # fixme -- need to set size of fonts and radio buttons larger
    sg.set_options(scaling=2.0)

    layout = [[sg.Text('Choose a task then press RUN')],

              [sg.HorizontalSeparator()],
              [sg.Text(spaces), sg.Radio('stand',                  group_id=1, key='-SN-')],
              [sg.Text(spaces), sg.Radio('test - move XYZ',        group_id=1, key='-XYZ-')],
              [sg.Text(spaces), sg.Radio('test - move joint',      group_id=1, key='-MJ-')],
              [sg.Text(spaces), sg.Radio('test - walk 10 seconds', group_id=1, key='-W10-')],
              [sg.HorizontalSeparator()],
              [sg.Text(spaces), sg.Button('do it!', key='-RUN-')],
              [sg.Text('Status:'),sg.Text(waiting, size=40, key='-STATUS-')],

              [sg.HorizontalSeparator()],
              [sg.Text(spaces), sg.Button('Exit')]
              ]

    window_title = 'Spot -- level 1'
    if config.ui_web_gui:
        window = sg.Window(window_title, layout, web_port=2222, web_start_browser=False)
    elif config.ui_x11_gui:
        window = sg.Window(window_title, layout)

    while True:  # Event Loop
        event, values = window.read()
        # print(event, values)

        if event in (None, 'Exit'):
            break

        if event == '-RUN-':
            if values['-SN-']:
                window.Hide()
                #stand.start()
                window.UnHide()

            elif values['-XYZ-']:
                window.Hide()
                xyz.run_gui()
                window.UnHide()

            elif values['-MJ-']:
                window.Hide()
                joints.run_test_gui()
                window.UnHide()

            elif values['-W10-']:
                window.Hide()
                robot.walk_test()
                window.UnHide()

    window.close()


def start_ui_level1():
    """ Allow the user to choose what "program" to run, then start it up and wait for it to end"""

    manage_level1_webx11_gui()
    log.debug('level1 gui END')

    # TBD Need to start a task to watch the buttons and maintain LCD screen on robot
    return

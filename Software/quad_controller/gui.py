import config

if config.ui_web_gui:
    import PySimpleGUIWeb as sg
elif config.ui_x11_gui:
    import PySimpleGUI as sg

import global_state as gs


"""Provides a user interface for contoling the robot and viewing its state.

Basically the GIU is a way for the user to access and set a few global veriables
that define the desires pose and velocity and other operating parameters

Multiple intefaces can be active at the same time

There are at least these interfaces
1) GUI web or X11 based
2) PS gamepad controller
"""


def run_gui():
    global StepPeriod
    global StepLength
    global StepHeight
    global Gait

    layout = [[sg.Text('Spot Micro SMM1')],

              [sg.Text('seconds per step '),
               sg.Text('2.0', key='freq_lable'),
               sg.Slider(range=(0.5, 4.0), resolution=0.1, default_value=2.0,
                         enable_events=True, key='-step_freq-')],

              [sg.Text('step length '),
               sg.Text('0.0', key='len_lable'),
               sg.Slider(range=(0.0, 0.100), resolution=0.01, default_value=0.0,
                         enable_events=True, key='-length-')],

              [sg.Text('step height '),
               sg.Text('0.0', key='ht_lable'),
               sg.Slider(range=(0.0, 0.050), resolution=0.01, default_value=0.0,
                         enable_events=True, key='-height-')],

              [sg.Button('STOP'),
               sg.Button('amble'),
               sg.Button('trot'),
               sg.Button('bound')],

              [sg.Button('Exit')]
              ]

    if config.web_gui:
        window = sg.Window('Spot Micro SMM1', layout, web_port=2222, web_start_browser=False)
    elif config.x11_gui:
        window = sg.Window('Spot Micro SMM1', layout)

    while True:  # Event Loop
        event, values = window.read()
        # print(event, values)

        if event in (None, 'Exit'):
            break

        if event == '-step_freq-':
            window['freq_lable'].Update(values['-step_freq-'])

            gs.GlobalLock.acquire()
            gs.StepPeriod = 1.0 / values['-step_freq-']
            gs.GlobalLock.release()

        if event == '-length-':
            window['len_lable'].Update(values['-length-'])

            gs.GlobalLock.acquire()
            gs.StepLength = values['-length-']
            gs.GlobalLock.release()

        if event == '-height-':
            window['ht_lable'].Update(values['-height-'])

            gs.GlobalLock.acquire()
            gs.StepHeight = values['-height-']
            gs.GlobalLock.release()

        if event == 'STOP':
            gs.GlobalLock.acquire()
            gs.Gait = 'STOP'
            gs.GlobalLock.release()

        if event == 'amble':
            gs.GlobalLock.acquire()
            gs.Gait = 'amble'
            gs.GlobalLock.release()

        if event == 'trot':
            gs.GlobalLock.acquire()
            gs.Gait = 'trot'
            gs.GlobalLock.release()

        if event == 'bound':
            gs.GlobalLock.acquire()
            gs.Gait = 'bound'
            gs.GlobalLock.release()

    window.close()

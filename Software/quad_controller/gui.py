#import PySimpleGUI as sg
import PySimpleGUIWeb as sg
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

              [sg.Text('step period '),
               sg.Text('0.0', key='per_lable'),
               sg.Slider(range=(0.0, 1.0), resolution=0.01, default_value=0.0,
                         enable_events=True, key='-period-')],

              [sg.Text('step length '),
               sg.Text('0.0', key='len_lable'),
               sg.Slider(range=(0.0, 1.0), resolution=0.01, default_value=0.0,
                         enable_events=True, key='-length-')],

              [sg.Text('step height '),
               sg.Text('0.0', key='ht_lable'),
               sg.Slider(range=(0.0, 1.0), resolution=0.01, default_value=0.0,
                         enable_events=True, key='-height-')],

              [sg.Button('STOP'),
               sg.Button('amble'),
               sg.Button('trot'),
               sg.Button('bound')],

              [sg.Button('Exit')]
              ]

    window = sg.Window('Spot Micro SMM1', layout, web_port=2222, web_start_browser=False)

    while True:  # Event Loop
        event, values = window.read()
        # print(event, values)

        if event in (None, 'Exit'):
            break

        if event == '-period-':
            window['per_lable'].Update(values['-period-'])

            gs.GlobalLock.acquire()
            StepPeriod = values['-period-']
            gs.GlobalLock.release()

        if event == '-length-':
            window['len_lable'].Update(values['-length-'])

            gs.GlobalLock.acquire()
            StepLength = values['-length-']
            gs.GlobalLock.release()

        if event == '-height-':
            window['ht_lable'].Update(values['-height-'])

            gs.GlobalLock.acquire()
            StepHeight = values['-height-']
            gs.GlobalLock.release()

        if event == 'STOP':
            gs.GlobalLock.acquire()
            Gait = 'STOP'
            gs.GlobalLock.release()

        if event == 'amble':
            gs.GlobalLock.acquire()
            Gait = 'amble'
            gs.GlobalLock.release()

        if event == 'trot':
            gs.GlobalLock.acquire()
            Gait = 'trot'
            gs.GlobalLock.release()

        if event == 'bound':
            gs.GlobalLock.acquire()
            Gait = 'bound'
            gs.GlobalLock.release()

        window.close()

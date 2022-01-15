import config

if config.ui_web_gui:
    import PySimpleGUIWeb as sg
elif config.ui_x11_gui:
    import PySimpleGUI as sg

import leg
import servo_shim as ss

global y_direction
y_direction = 0.0
global active_leg
active_leg = 'none'

test_leg = leg.Leg()

global Servo
Servo = ss.servo_shim()


def move(x,y,z):

    global y_direction
    global active_leg
    global Servo

    if active_leg == 'none':
        return

    joint1, joint2, joint3 = test_leg.ik3d(x, y*y_direction, z)

    # Send the angle to the servo motors
    if active_leg == 'lr':
        print('move x,y,z,1,2,3', x, y, z, joint1, joint2, joint3)
        Servo.set_angle( 8, joint1)
        Servo.set_angle( 9, joint2)
        Servo.set_angle(10, joint3)
    

def run_gui():

    global y_direction
    global active_leg

    default_x =  0.0
    default_y =  0.050
    default_z = -0.100
    foot_x = default_x
    foot_y = default_y
    foot_z = default_z
    
    # Init servos
    

    layout = [[sg.Text('Spot Micro SMM1')],

              [sg.Button('Left Front'), sg.Button('Right Front')],
              [sg.Button('Left Rear'), sg.Button('Right Rear')],
              [sg.Text('Active Leg:'), sg.Text('none', key='active_leg_key')],

              [sg.Text('X mm '),
               sg.Text('0.0', key='x_text'),
               sg.Slider(range=(-100.0, 100.0), resolution=1.0, default_value=0.0,
                         enable_events=True, key='-X-')],

              [sg.Text('Y mm'),
               sg.Text('50.0', key='y_text'),
               sg.Slider(range=(-100, 100), resolution=1.0, default_value=50.0,
                         enable_events=True, key='-Y-')],

              [sg.Text('Z mm'),
               sg.Text('-100.0', key='z_text'),
               sg.Slider(range=(-250.0, 100), resolution=1.0, default_value=-100.0,
                         enable_events=True, key='-Z-')],

              [sg.Button('PLOT'), sg.Button('Exit')]
              ]

    if config.ui_web_gui:
        window = sg.Window('Spot Micro SMM1', layout, web_port=2222, web_start_browser=False)
    elif config.ui_x11_gui:
        window = sg.Window('Spot Micro SMM1', layout)

    while True:  # Event Loop
        event, values = window.read()
        # print(event, values)

        if event in (None, 'Exit'):
            break

        if event == '-X-':
            window['x_text'].Update(values['-X-'])
            foot_x = values['-X-'] / 1000.0
            move(foot_x, foot_y, foot_z)

        if event == '-Y-':
            window['y_text'].Update(values['-Y-'])
            foot_y = values['-Y-'] / 1000.0
            move(foot_x, foot_y, foot_z)

        if event == '-Z-':
            window['z_text'].Update(values['-Z-'])
            foot_z = values['-Z-'] / 1000.0
            move(foot_x, foot_y, foot_z)

        if event == 'Left Front':
            window['active_leg_key'].Update('Left Front ')
            active_leg = 'lf'
            y_direction = 1

        if event == 'Right Front':
            window['active_leg_key'].Update('Right Front')
            active_leg = 'rf'
            y_direction = -1

        if event == 'Left Rear':
            window['active_leg_key'].Update('Left Rear  ')
            active_leg = 'lr'
            y_direction = 1

        if event == 'Right Rear':
            window['active_leg_key'].Update('Right Rear ')
            active_leg = 'rr'
            y_direction = -1

        if event == 'PLOT':
            pass

    window.close()

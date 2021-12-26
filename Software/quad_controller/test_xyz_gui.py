import config

if config.web_gui:
    import PySimpleGUIWeb as sg
elif config.x11_gui:
    import PySimpleGUI as sg

import leg
import servo_shim as ss

test_leg = leg.Leg()
Servo = ss.servo_shim()


def move(x,y,z):

    # fixme - Y (left/right) is currently fixed
    ft_point = (x, z)
    joint1, joint2, joint3 = test_leg.ik3d(x, y, z)

    # Send the angle to the servo motors
    Servo.set_angle(12, joint1)
    Servo.set_angle(13, joint2)
    Servo.set_angle(14, joint3)

    print('move x,z,hip,knee', x, z, angle_hip, angle_knee)

def run_gui():

    default_x =  0.0
    default_y =  0.050
    default_z = -0.100
    foot_x = default_x
    foot_y = default_y
    foot_z = default_z

    layout = [[sg.Text('Spot Micro SMM1')],

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

    window.close()

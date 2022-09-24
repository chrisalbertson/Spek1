import logging
import math
log = logging.getLogger(__name__)

import config
if config.ui_web_gui:
    import PySimpleGUIWeb as sg
elif config.ui_x11_gui:
    import PySimpleGUI as sg

import robot

"""
creating the Robot object starts a Python thread that runs continously
until the program terminates.  Later we make funtion calls into the 
Robot instnce 'r' to make the robot do things.  The funtins calls mostly
just write to shared memory """
r = robot.Robot()


def run_gui():
    walk_speed = 0.0
    walk_turnrate = 0.0
    stance_adjust = 0.0
    body_height_default = 180.0
    body_height = body_height_default
    body_right = 0.0
    body_forward = 0.0
    body_pitch = 0.0
    body_roll = 0.0
    body_yaw = 0.0

    if config.ui_x11_gui:
        indent = ((20,0),(0,0)) # padding on left side only
    
        drive_elements = \
            [   [sg.Text('Speed (m/s)', size=(25,1), pad=indent),
                 sg.Text(str(walk_speed), key='-SPEED_TEXT-'),
                 sg.Slider(range = (0.0, config.max_forward_speed),
                           default_value=walk_speed,
                           resolution=config.max_forward_speed / 20.0,
                           enable_events=True,
                           orientation='h',
                           key='-SPEED-')],

                [sg.Text('Steering', size=(25,1), pad=indent),
                 sg.Text(str(walk_turnrate), key='-TURN_TEXT-'),
                 sg.Slider(range = (-0.5, 0.5),
                           default_value=walk_turnrate,
                           resolution=0.01,
                           enable_events=True,
                           orientation='h',
                           key='-TURN-')],

                [sg.Text('Stance Adjust', size=(25,1), pad=indent),
                 sg.Text(str(stance_adjust), key='-STANCE_ADJUST_TEXT-'),
                 sg.Slider(range = (-10.0, 10.0),
                           default_value=stance_adjust,
                           resolution=1.0,
                           enable_events=True,
                           orientation='h',
                           key='-STANCE_ADJUST-')]
            ]
    
        translation_elements = \
            [   [sg.Text('Height', size=(25,1), pad=indent),
                 sg.Text(str(body_height), key='-HEIGHT_TEXT-'),
                 sg.Slider(range = (120.0, 240.0),
                           default_value=body_height_default,
                           resolution=1.0,
                           enable_events=True,
                           orientation='h',
                           key='-HEIGHT-')],

                [sg.Text('Right/Left', size=(25,1), pad=indent),
                 sg.Text(str(body_right), key='-RIGHT_TEXT-'),
                 sg.Slider(range = (-80.0, 80.0),
                           default_value=body_right,
                           resolution=1.0,
                           enable_events=True,
                           orientation='h',
                           key='-RIGHT-')],


                [sg.Text('Fore/Aft', size=(25,1), pad=indent),
                 sg.Text(str(body_forward), key='-FORWARD_TEXT-'),
                 sg.Slider(range = (-80.0, 80.0),
                           default_value=body_forward,
                           resolution=1.0,
                           enable_events=True,
                           orientation='h',
                           key='-FORWARD-')],
                           
               [sg.Button('Reset Translations', size=20, key='-RESET_TRANSLATIONS-')]
            ]
    
        
        rotation_elements = \
            [   [sg.Text('Roll', size=(25,1), pad=indent),
                 sg.Text(str(body_roll), key='-ROLL_TEXT-'),
                 sg.Slider(range = (-20.0, 20.0),
                           default_value=body_roll,
                           resolution=1.0,
                           enable_events=True,
                           orientation='h',
                           key='-ROLL-')],


                [sg.Text('Pitch', size=(25,1), pad=indent),
                 sg.Text(str(body_pitch), key='-PITCH_TEXT-'),
                 sg.Slider(range = (-20.0, 20.0),
                           default_value=body_pitch,
                           resolution=1.0,
                           enable_events=True,
                           orientation='h',
                           key='-PITCH-')],


                [sg.Text('yaw', size=(25,1), pad=indent),
                 sg.Text(str(body_yaw), key='-YAW_TEXT-'),
                 sg.Slider(range = (-20.0, 20.0),
                           default_value=body_yaw,
                           resolution=1.0,
                           enable_events=True,
                           orientation='h',
                           key='-YAW-')],
                           
               [sg.Button('Reset Rotations', size=20, key='-RESET_ROTATIONS-')]
            ]

    
        layout = \
            [      
                [sg.Frame('Driving', drive_elements)],
                [sg.Frame('Body Translation', translation_elements)],
                [sg.Frame('Body Rotation',    rotation_elements)],


                [sg.Button('WALK', size=10, key='-WALK-'),
                 sg.Button('STOP', size=10, key='-STOP-'),
                 sg.Button('PANIC', size=10, key='-PANIC-')]
             ]
             
    elif config.ui_web_gui:
        layout_web = [[sg.Text('Speed (m/s)'),
               sg.Text(str(walk_speed), key='-SPEED_TEXT-'),
               sg.Slider(range=(0.0, config.max_forward_speed),
                         default_value=walk_speed,
                         resolution=config.max_forward_speed / 20.0,
                         enable_events=True,
                         orientation='h',
                         key='-SPEED-')],

              [sg.Text('Steering'),
               sg.Text(str(walk_turnrate), key='-TURN_TEXT-'),
               sg.Slider(range=(-1.0, 1.0),
                         default_value=walk_turnrate,
                         resolution=0.01,
                         enable_events=True,
                         orientation='h',
                         key='-TURN-')],

              [sg.Button('WALK', key='-WALK-'),
               sg.Button('STOP', key='-STOP-'),
               sg.Button('PANIC', key='-PANIC-')]
              ]

    if config.ui_web_gui:
        window = sg.Window('Walk Web GUI', layout_web, web_port=2222, web_start_browser=False)
    elif config.ui_x11_gui:
        window = sg.Window('Walk GUI', layout)

    while True:  # Event Loop
        event, values = window.read()

        if event in (None, 'Exit'):
            break

        elif event == '-SPEED-':
            walk_speed = float(values['-SPEED-'])
            window['-SPEED_TEXT-'].update(str(walk_speed))
            r.set_velocity(walk_speed)

        elif event == '-TURN-':
            walk_turnrate = values['-TURN-']
            window['-TURN_TEXT-'].update(str(walk_turnrate))
            r.set_turnrate(walk_turnrate)

        elif event == '-STANCE_ADJUST-':
            stance_adjust = values['-STANCE_ADJUST-']
            window['-STANCE_ADJUST_TEXT-'].update(str(stance_adjust))
            #r.adjust_stance(stance_adjust)

        elif event == '-HEIGHT-':
            body_height = values['-HEIGHT-']
            window['-HEIGHT_TEXT-'].update(str(body_height))
            r.set_body_center((body_forward/1000.0, body_right/1000.0, body_height/1000.0))

        elif event == '-RIGHT-':
            body_right = values['-RIGHT-']
            window['-RIGHT_TEXT-'].update(str(body_right))
            r.set_body_center((body_forward/1000.0, body_right/1000.0, body_height/1000.0))

        elif event == '-FORWARD-':
            body_forward = values['-FORWARD-']
            window['-FORWARD_TEXT-'].update(str(body_forward))
            r.set_body_center((body_forward/1000.0, body_right/1000.0, body_height/1000.0))
            
        
            
        elif event == '-RESET_TRANSLATIONS-':
            body_height  = body_height_default
            body_right   = 0.0
            body_forward = 0.0
            window['-HEIGHT_TEXT-'].update(str(body_height))
            window['-RIGHT_TEXT-'].update(str(body_right))
            window['-FORWARD_TEXT-'].update(str(body_forward))
            window['-HEIGHT-'].update(body_height)
            window['-RIGHT-'].update(body_right)
            window['-FORWARD-'].update(body_forward)
            r.set_body_center((body_forward/1000.0, body_right/1000.0, body_height/1000.0))

        elif event == '-ROLL-':
            body_roll = values['-ROLL-']
            window['-ROLL_TEXT-'].update(str(body_roll))
            r.set_body_angles((math.radians(body_roll), math.radians(body_pitch), math.radians(body_yaw)))

        elif event == '-PITCH-':
            body_pitch = values['-PITCH-']
            window['-PITCH_TEXT-'].update(str(body_pitch))
            r.set_body_angles((math.radians(body_roll), math.radians(body_pitch), math.radians(body_yaw)))

        elif event == '-YAW-':
            body_yaw = values['-YAW-']
            window['-YAW_TEXT-'].update(str(body_yaw))
            r.set_body_angles((math.radians(body_roll), math.radians(body_pitch), math.radians(body_yaw)))
            
        elif event == '-RESET_ROTATIONS-':
            body_roll  = 0.0
            body_pitch = 0.0
            body_yaw   = 0.0
            window['-ROLL_TEXT-'].update(str(body_roll))
            window['-PITCH_TEXT-'].update(str(body_pitch))
            window['-YAW_TEXT-'].update(str(body_yaw))
            window['-ROLL-'].update(body_roll)
            window['-PITCH-'].update(body_pitch)
            window['-YAW-'].update(body_yaw)
            r.set_body_angles((math.radians(body_roll), math.radians(body_pitch), math.radians(body_yaw)))
            
        elif event == '-WALK-':
            walk_speed = float(values['-SPEED-'])
            r.walk(walk_speed)

        elif event == '-STOP-':
            r.stop_natural()

        elif event == '-PANIC-':
            r.stop_panic()
            break

    window.close()
    r.kill()
    return


if __name__ == "__main__":
    logging.basicConfig(filename='quad_controller.log',
                        filemode='w',
                        level=logging.WARNING, )

    config.UsePCA9685Hardware = True

    log.info('Starting GUI')
    run_gui()
    log.info('normal termination')

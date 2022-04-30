import logging
log = logging.getLogger(__name__)

import config
if config.ui_web_gui:
    import PySimpleGUIWeb as sg
elif config.ui_x11_gui:
    import PySimpleGUI as sg

import robot

r = robot.Robot()

def run_gui():
    walk_speed = 0.0
    walk_turnrate = 0.0
    walk_height = 180.0

    if config.ui_x11_gui:
        layout = [  [sg.Text('Speed (m/s)', size=(25,1)),
                 sg.Text(str(walk_speed), key='-SPEED_TEXT-'),
                 sg.Slider(range = (0.0, config.max_forward_speed),
                           default_value=walk_speed,
                           resolution=config.max_forward_speed / 20.0,
                           enable_events=True,
                           orientation='h',
                           key='-SPEED-')],

                [sg.Text('Steering', size=(25,1)),
                 sg.Text(str(walk_turnrate), key='-TURN_TEXT-'),
                 sg.Slider(range = (-1.0, 1.0),
                           default_value=walk_turnrate,
                           resolution=0.01,
                           enable_events=True,
                           orientation='h',
                           key='-TURN-')],

                [sg.Text('Height', size=(25,1)),
                 sg.Text(str(walk_turnrate), key='-HEIGHT_TEXT-'),
                 sg.Slider(range = (120.0, 220.0),
                           default_value=walk_height,
                           resolution=1.0,
                           enable_events=True,
                           orientation='h',
                           key='-HEIGHT-')],

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

        elif event == '-HEIGHT-':
            walk_height = values['-HEIGHT-']
            window['-HEIGHT_TEXT-'].update(str(walk_height))
            r.set_body_center((0.0, 0.0, walk_height / 1000.0))

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

""" Hello World program for one-quarter dog robot.

My first mile stone in building a robot dog is is assemble one leg on a
fixed test stand and get it to move from a Python script.  This is about
the simplest GIU Python script possible and is used to test the hardwre
and the range of motion.
"""

import PySimpleGUI as sg
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)
kit.servo[12].set_pulse_width_range(min_pulse=500, max_pulse=2500)
kit.servo[13].set_pulse_width_range(min_pulse=500, max_pulse=2500)
kit.servo[14].set_pulse_width_range(min_pulse=500, max_pulse=2500)
kit.servo[14].actuation_range = 180


def spot_sit():
    kit.servo[12].angle = 90
    kit.servo[13].angle = 180
    kit.servo[14].angle = 45
    window.Element('slider1').update(90)
    window.Element('slider2').update(180)
    window.Element('slider3').update(45)

def spot_stand():
    kit.servo[12].angle = 90
    kit.servo[13].angle = 90
    kit.servo[14].angle = 130
    window.Element('slider1').update(90)
    window.Element('slider2').update(90)
    window.Element('slider3').update(130)

def spot_walk():
    sg.popup('Spot does not know how to walk')

sg.theme('DarkAmber')

layout = [  [sg.Text('Shoulder'), sg.Text('Upper Leg'),sg.Text('Lower Leg') ],

            [   sg.Slider(range =(0,180), default_value=90, orientation='v', enable_events = True, key='slider1'),
                sg.Slider(range =(0,180), default_value=90, orientation='v', enable_events = True, key='slider2' ),
                sg.Slider(range =(0,180), default_value=90, orientation='v', enable_events = True, key='slider3' )],

            [sg.HorizontalSeparator()],
            [sg.Button('sit'), sg.Button('stand'), sg.Button('walk')],
            [sg.Button('Min Limits'), sg.Button('50%'), sg.Button('Max Limits')],
            [sg.HorizontalSeparator()],
            [sg.Button('Exit')] ]

# Create the Window
window = sg.Window('One Leg Pose Demo', layout)

# Initialize the servos to default values
kit.servo[12].fraction = 0.50
kit.servo[13].fraction = 0.50
kit.servo[14].fraction = 0.50

# Event Loop to process "events" and get the "values" of the inputs
while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED or event == 'Exit': 
        break

    if event == 'sit':
        spot_sit()
 
    if event == 'stand':
        spot_stand()
    
    if event == 'walk':
        spot_walk()

    if event == '50%':
        kit.servo[12].fraction = 0.50
        kit.servo[13].fraction = 0.50
        kit.servo[14].fraction = 0.50
        window.Element('slider1').update(90)
        window.Element('slider2').update(90)
        window.Element('slider3').update(90)

    if event == 'Max Limits':
        kit.servo[12].fraction = 1.0
        kit.servo[13].fraction = 1.0
        kit.servo[14].fraction = 1.0
        window.Element('slider1').update(180)
        window.Element('slider2').update(180)
        window.Element('slider3').update(180)

    if event == 'Min Limits':
        kit.servo[12].fraction = 0.0
        kit.servo[13].fraction = 0.0
        kit.servo[14].fraction = 0.0
        window.Element('slider1').update(0)
        window.Element('slider2').update(0)
        window.Element('slider3').update(0)


    if event == 'slider1':
        kit.servo[12].angle = values['slider1']

    if event == 'slider2':
        kit.servo[13].angle = values['slider2']

    if event == 'slider3':
        kit.servo[14].angle = values['slider3']
    
    
window.close()
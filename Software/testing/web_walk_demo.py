import PySimpleGUIWeb as sg
import threading 
import time
import math

GotHardware = True
if (GotHardware):
    from adafruit_servokit import ServoKit


""" GLOBAL VARIABLES
    These are used by both the GUI and control loop task
"""
GlobalLock = threading.Lock()

global StepPeriod
global StepLength
global StepHeight
global Gait

StepPeriod = 0.0
StepLength = 0.0
StepHeight = 0.0
Gait = 'STOP'

"""
    END GLOBAL VARIABLES
"""
if (GotHardware):
    kit = ServoKit(channels=16, frequency=325)
    kit.servo[12].set_pulse_width_range(min_pulse=500, max_pulse=2500)
    kit.servo[13].set_pulse_width_range(min_pulse=500, max_pulse=2500)
    kit.servo[14].set_pulse_width_range(min_pulse=500, max_pulse=2500)
    


def robot_control_loop():


    global StepPeriod
    global StepLength
    global StepHeight
    global Gait

    #needed only for simple test
    servo14 = 90.0
    servo13 = 90.0
    bump = 0.1
    incr14 = bump
    incr13 = bump
    
    
    next_heartbeat = 0.0

    loop_frequency = 150.0    # Hz.
    loop_period = 1.0 / loop_frequency
    while(1):
        tick_start = time.time()
            
        # grab globals
        GlobalLock.acquire() 
        step_period = StepPeriod
        step_length = StepLength
        step_height = StepHeight
        gait = Gait
        GlobalLock.release()
        
        # Is it time to heartbeat yet?
        if (tick_start > next_heartbeat):
            next_heartbeat = tick_start + 1.0
            # TBD - blink a LED
            print('Heatbeat at', tick_start, step_period, step_length, step_height, gait)
            
        # TBD Do stuff here
        # now just a bunch of neary random movments
        
        servo14 += incr14
        servo13 += incr13
        
        accel = 1 + abs(math.sin(servo14 * 0.017453))
        if (servo14 > 179.0):
            incr14 = -bump * 2.5 * accel
        if (servo14 < 1.0):
            incr14 = bump * 2 *accel
            
        accel = 1 + (0.5*math.sin(servo13 * 0.017453))    
        if (servo13 > 175.0):
            incr13 = -bump * 1.1 * accel
        if (servo13 < 5.0):
            incr13 = bump * accel
            
        if(GotHardware):
            kit.servo[14].angle = servo14
            kit.servo[13].angle = servo13
        else:
            print("simulation: set servo13 14 =", servo13, servo14)
    
    
         # Wait a little less than loop_period
        tick_elapsed = time.time() - tick_start
        sleep_time = loop_period - tick_elapsed
        if (sleep_time < 0.003):
            print('ALARM - Loop elaspsed time', tick_elapsed)
        else:	
            time.sleep(sleep_time)

    
def run_gui():  

    global StepPeriod
    global StepLength
    global StepHeight
    global Gait 

    layout = [  [sg.Text('Spot Micro SMM1')],

                [sg.Text('step period '),
                 sg.Text('0.0', key='per_lable'),
                 sg.Slider(range =(0.0,1.0), resolution=0.01, default_value=0.0, enable_events = True, key='-period-')],

                [sg.Text('step length '),
                 sg.Text('0.0', key='len_lable'),
                 sg.Slider(range =(0.0,1.0), resolution=0.01, default_value=0.0, enable_events = True, key='-length-')],

                [sg.Text('step height '),
                 sg.Text('0.0', key='ht_lable'),
                 sg.Slider(range =(0.0,1.0), resolution=0.01, default_value=0.0, enable_events = True, key='-height-')],
                 
                [sg.Button('STOP'), 
                 sg.Button('amble'),
                 sg.Button('trot'),
                 sg.Button('bound')],
                
                [sg.Button('Exit')]  
            ]

    window = sg.Window('Spot Micro SMM1', layout, web_port=2222, web_start_browser=False)

    while True:             # Event Loop
        event, values = window.read()
        # print(event, values)
        
        if event in (None, 'Exit'):
            break
            
        if event == '-period-':
            window['per_lable'].Update(values['-period-'])
            
            GlobalLock.acquire() 
            StepPeriod = values['-period-']
            GlobalLock.release()
            
        if event == '-length-':
            window['len_lable'].Update(values['-length-'])
            
            GlobalLock.acquire() 
            StepLength = values['-length-']
            GlobalLock.release()
            
        if event == '-height-':
            window['ht_lable'].Update(values['-height-'])
            
            GlobalLock.acquire() 
            StepHeight = values['-height-']
            GlobalLock.release()
            
        if event == 'STOP':
            
            GlobalLock.acquire() 
            Gait = 'STOP'
            GlobalLock.release()
            
        if event == 'amble':
            
            GlobalLock.acquire() 
            Gait = 'amble'
            GlobalLock.release()
            
        if event == 'trot':
            
            GlobalLock.acquire() 
            Gait = 'trot'
            GlobalLock.release()
            
        if event == 'bound':
            
            GlobalLock.acquire() 
            Gait = 'bound'
            GlobalLock.release()
            
    window.close()

def start_tasks():
    
    robot_thread = threading.Thread(target=robot_control_loop, args=(), daemon=True)
    
    robot_thread.start()
    run_gui()

if __name__ == '__main__':
    start_tasks()


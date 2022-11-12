qimport numpy as np
import config

if config.ui_web_gui:
    import PySimpleGUIWeb as sg
elif config.ui_x11_gui:
    import PySimpleGUI as sg

import sm_kinematics
import joints

import logging
log = logging.getLogger(__name__)


joint = joints.ServoShim()


def move(x,y,z):


    global active_leg
    global joint

    if active_leg == 'none':
        return


def run_gui():

    smk = sm_kinematics.Kinematic()

    # Define a neutral stance.  Units are Meters.
    body_x = 0.0
    body_y = 0.0
    body_z = 0.0

    body_roll  = 0.0
    body_pitch = 0.0
    body_yaw   = 0.0

    vertical = 0.180
    stance_width = 0.150
    stance_length = 0.260
    stance_halfwidth = stance_width / 2.0
    stance_halflength = stance_length / 2.0

    lfx = stance_halflength
    lfy = -vertical
    lfz = stance_halfwidth

    rfx = stance_halflength
    rfy = -vertical
    rfz = -stance_halfwidth

    lrx = -stance_halflength
    lry = -vertical
    lrz = stance_halfwidth

    rrx = -stance_halflength
    rry = -vertical
    rrz = -stance_halfwidth

    input_mm_sz: int = 8
    layout = [
              [sg.Text('           roll             pitch              yaw')],
              [sg.Text('Body'), sg.Input(body_roll,  key='-BRR-', size=input_mm_sz),
                                sg.Input(body_pitch, key='-BPR-', size=input_mm_sz),
                                sg.Input(body_yaw,   key='-BYR-', size=input_mm_sz)],

              [sg.Text('            X             Y               Z')],
              [sg.Text('Body'), sg.Input(body_x, key='-BX-', size=input_mm_sz),
                                sg.Input(body_y, key='-BY-', size=input_mm_sz),
                                sg.Input(body_z, key='-BZ-', size=input_mm_sz)],

              [sg.Text('            X              Y              Z')],
              [sg.Text('LF '), sg.Input(lfx, key='-LFX-', size=input_mm_sz),
                               sg.Input(lfy, key='-LFY-', size=input_mm_sz),
                               sg.Input(lfz, key='-LFZ-', size=input_mm_sz)],

              [sg.Text('RF '), sg.Input(rfx, key='-RFX-', size=input_mm_sz),
                               sg.Input(rfy, key='-RFY-', size=input_mm_sz),
                               sg.Input(rfz, key='-RFZ-', size=input_mm_sz)],

              [sg.Text('LR '), sg.Input(lrx, key='-LRX-', size=input_mm_sz),
                               sg.Input(lry, key='-LRY-', size=input_mm_sz),
                               sg.Input(lrz, key='-LRZ-', size=input_mm_sz)],

              [sg.Text('RR '), sg.Input(rrx, key='-RRX-', size=input_mm_sz),
                               sg.Input(rry, key='-RRY-', size=input_mm_sz),
                               sg.Input(rrz, key='-RRZ-', size=input_mm_sz)],

              [sg.Button('Compute', key='-COMPUTE-')],
              [sg.Text('', key='-IK_solution-', size=(50, 5))],
              [sg.Button('Move', key='-MOVE-')],
              [sg.Text('', key='-ServoAngles-', size=(50, 5))],
             ]

    if config.ui_web_gui:
        window = sg.Window('Spot Micro', layout, web_port=2222, web_start_browser=False)
    elif config.ui_x11_gui:
        window = sg.Window('Spot Micro', layout)

    while True:  # Event Loop
        event, values = window.read()
        # print(event, values)

        if event in (None, 'Exit'):
            break

        if event == '-COMPUTE-':
            body_x = float(values['-BX-'])
            body_y = float(values['-BY-'])
            body_z = float(values['-BZ-'])

            body_roll =  float(values['-BRR-'])
            body_pitch = float(values['-BPR-'])
            body_yaw =   float(values['-BYR-'])

            lfx = float(values['-LFX-'])
            lfy = float(values['-LFY-'])
            lfz = float(values['-LFZ-'])

            rfx = float(values['-RFX-'])
            rfy = float(values['-RFY-'])
            rfz = float(values['-RFZ-'])

            lrx = float(values['-LRX-'])
            lry = float(values['-LRY-'])
            lrz = float(values['-LRZ-'])

            rrx = float(values['-RRX-'])
            rry = float(values['-RRY-'])
            rrz = float(values['-RRZ-'])

            # Distances of each foot from body center
            #                   fore/aft         height       left/right
            Lp = np.array([[lfx, lfy, lfz, 1.],   # LF
                           [rfx, rfy, rfz, 1.],   # RF
                           [lrx, lry, lrz, 1.],   # LR
                           [rrx, rry, rrz, 1.]])  # RR
            if abs(np.amax(Lp[:, :3])) > 0.300:
                sg.popup_error('An xyz value is out of range.')
                continue

            body_angles = (body_roll, body_pitch, body_yaw)
            body_center = (body_x, body_y, body_z)
            joint_angles = smk.calcIK(Lp, body_angles, body_center)
            lfth1, lfth2, lfth3 = joint_angles[0, :]
            rfth1, rfth2, rfth3 = joint_angles[1, :]
            lrth1, lrth2, lrth3 = joint_angles[2, :]
            rrth1, rrth2, rrth3 = joint_angles[3, :]
            angle_str = '     Theta1    Theta2    Theta3\n' \
                        'LF {0:8.2f}, {1:8.2f}, {2:8.2f}\n' \
                        'RF {3:8.2f}, {4:8.2f}, {5:8.2f}\n' \
                        'LR {6:8.2f}, {7:8.2f}, {8:8.2f}\n' \
                        'RR {9:8.2f}, {10:8.2f}, {11:8.2f}\n'.format(
                        lfth1, lfth2, lfth3,
                        rfth1, rfth2, rfth3,
                        lrth1, lrth2, lrth3,
                        rrth1, rrth2, rrth3)
            window['-IK_solution-'].update(angle_str)

        if event == '-MOVE-':
            lfsa1 = joint.set_angle( 0, lfth1)
            lfsa2 = joint.set_angle( 1, lfth2)
            lfsa3 = joint.set_angle( 2, lfth3)
            rfsa1 = joint.set_angle( 4, rfth1)
            rfsa2 = joint.set_angle( 5, rfth2)
            rfsa3 = joint.set_angle( 6, rfth3)
            lrsa1 = joint.set_angle( 8, lrth1)
            lrsa2 = joint.set_angle( 9, lrth2)
            lrsa3 = joint.set_angle(10, lrth3)
            rrsa1 = joint.set_angle(12, rrth1)
            rrsa2 = joint.set_angle(13, rrth2)
            rrsa3 = joint.set_angle(14, rrth3)
            servo_str = '   shoulder    upper      lower\n' \
                        'LF {0:8.2f}, {1:8.2f}, {2:8.2f}\n' \
                        'RF {3:8.2f}, {4:8.2f}, {5:8.2f}\n' \
                        'LR {6:8.2f}, {7:8.2f}, {8:8.2f}\n' \
                        'RR {9:8.2f}, {10:8.2f}, {11:8.2f}\n'.format(
                        lfsa1, lfsa2, lfsa3,
                        rfsa1, rfsa2, rfsa3,
                        lrsa1, lrsa2, lrsa3,
                        rrsa1, rrsa2, rrsa3)

            window['-ServoAngles-'].update(servo_str)

    window.close()

if __name__ == "__main__":
    logging.basicConfig(filename='quad_controller.log',
                        filemode='w',
                        level=logging.DEBUG, )

    run_gui()
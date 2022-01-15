# Controller for Spot Micro
import logging
log = logging.getLogger(__name__)

import argparse
import threading

import config
import global_state as gs
import gui
import test_xyz_gui
import walk_loop as loop
import user_interface as ui




def start_tasks():

    log.info('Starting Quad Controller')
    # get the "constant" parameters


    # init the servos
    # init the legs
    # init the body
    # Start the real-time control loop task
    robot_thread = threading.Thread(target=loop.control_loop, args=(), daemon=True)
    robot_thread.start()

    # Start the GUI
    gui.run_gui()

if __name__ == '__main__':

    print('THERE IS NO WAY THIS CAN BE CONSIDERED WORKING. It is just place holders and dummy code')
    # TBD process arguments  -w for web gui or -h for hardware of not

    parser = argparse.ArgumentParser(description='Quadruped Controller')
    parser.add_argument('--sym', action='store_true', default=False,
                        help='Run in simulation mode, do not access robot hardware.')
    parser.add_argument('--web', action='store_true', default=False,
                        help='Run the web based GUI')
    parser.add_argument('--x11', action='store_true', default=True,
                        help='Run the X11 based GUI')
    parser.add_argument('-log',
                        '--loglevel',
                        choices=['debug', 'info', 'warning'],
                        default='debug',
                        ## fixme default='warning',
                        help='Provide logging level. Example --loglevel debug, default=warning')

    args = parser.parse_args()

    if args.sym:
        config.GotHardware = False

    if args.web:
        config.ui_web_gui = True

    if args.x11:
        config.ui_x11_gui = True

    logging.basicConfig(filename='quad_controller.log', level=args.loglevel.upper())
    # logging.basicConfig(filename='quad_controller.log', level=logging.DEBUG)

    if config.GotHardware:
        log.info('Quad Controller starting...  Using real hardware')
    else:
        log.info('Quad Controller starting...  Running in simulation mode, Robot hardware not used')

    # Brings up all configured user interfaces.  The "level1" interface allows the user to select an
    # operating mode like "stand", "balance", "walk" or "follow remote"
    ui.start_ui_level1()

    # Brings up the real-time control loops, there look to global variables for input
    ## FIXME start_rt()

    log.info('Normal termination')

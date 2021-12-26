# Controller for Spot Micro
import logging
import argparse
import threading

import config
import global_state as gs
import gui
import test_xyz_gui
import walk_loop as loop




def start_tasks():

    logging.info('Starting Quad Controller')
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
    args = parser.parse_args()

    if args.sym:
        config.GotHardware = False

    # fixme -- mode needs to come from arg list.
    mode = 'test'
    # mode = 'normal'

    logging.basicConfig(filename='quad_controller.log', level=logging.INFO)
    # logging.basicConfig(filename='quad_controller.log', level=logging.DEBUG)

    if mode == 'normal':
        start_tasks()
    elif mode == 'test':
        test_xyz_gui.run_gui()

    logging.info('Normal termination')

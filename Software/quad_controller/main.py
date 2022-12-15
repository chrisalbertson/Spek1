"""Controller for Spot Micro"""
import logging
import argparse
import yappi

import config
import robot
import gui
#import game_controller

log = logging.getLogger(__name__)

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Quadruped Controller')
    parser.add_argument('--sym', action='store_true', default=False,
                        help='Run in simulation mode, do not access robot hardware.')
    parser.add_argument('--x11', action='store_true', default=True,
                        help='Run the X11 based GUI')
    parser.add_argument('--yappi', action='store_true', default=False,
                        help='Run yappi profiles to collect execution timing data.')
    parser.add_argument('--loglevel',
                        choices=['debug', 'info', 'warning'],
                        default='warning',
                        help='Specify logging level. Example --loglevel debug, default=warning')

    args = parser.parse_args()

    config.UsePCA9685Hardware = not args.sym
    config.ui_x11_gui         =     args.x11
    use_profiler              =     args.yappi

    logging.basicConfig(filename='quad_controller.log', level=args.loglevel.upper())

    if use_profiler:
        yappi.start()

    # creating the robot object also stars the real-tiime control task.
    r = robot.Robot()

    # Stand up then start up all configured user interfaces.
    r.stand()

    if config.ui_x11_gui:
       gui.run_gui(r)
    #game_controller.run_gamepad(r)

    if use_profiler:
        yappi.get_func_stats().print_all()
        yappi.get_thread_stats().print_all()

    log.info('Normal termination')

"""Controller for Spot Micro"""
import logging
import argparse

import config
import user_interface as ui

log = logging.getLogger(__name__)

if __name__ == '__main__':

    print('THERE IS NO WAY THIS CAN BE CONSIDERED WORKING. It is just place holders and dummy code')

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
                        default='debug',  #FIXME change detault to warning
                        help='Specify logging level. Example --loglevel debug, default=warning')

    args = parser.parse_args()

    if args.sym:
        config.GotHardware = False

    if args.web:
        config.ui_web_gui = True

    if args.x11:
        config.ui_x11_gui = True

    logging.basicConfig(filename='quad_controller.log', level=args.loglevel.upper())

    if config.GotHardware:
        log.info('Quad Controller starting...  Using real hardware')
    else:
        log.info('Quad Controller starting...  Robot hardware not used')

    # Brings up all configured user interfaces.  The "level1" interface allows the user to select an
    # operating mode like "stand", "balance", "walk" or "follow remote"
    ui.start_ui_level1()

    log.info('Normal termination')

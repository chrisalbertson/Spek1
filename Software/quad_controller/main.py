# Controller for Spot Micro
import logging

import global_state


def start_tasks():

    logging.info('Starting Quad Controller')
    # get the "constant" parameters

    # Start the GUI
    run_gui()

    # init the servos
    # init the legs
    # init the body
    # Start the real-time control loop task
    robot_thread = threading.Thread(target=robot_control_loop, args=(), daemon=True)
    robot_thread.start()


if __name__ == '__main__':

    print('THERE IS NO WAY THIS CAN BE CONSIDERED WORKING. It is just place holders and dummy code')
    # TBD process arguments  -w for web gui or -h for hardware of not
    logging.basicConfig(filename='quad_controller.log', level=logging.INFO)
    start_tasks()

    logging.info('Normal termination')

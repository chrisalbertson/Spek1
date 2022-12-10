"""Allows a locally connected game controller to operate the robot"""
import math

import Gamepad
import time
#import robot
import mqtt_remote

def map_pow(value: float) -> float:
    exp = 1.5

    if value >= 0.0:
        sign = 1.0
    else:
        sign = -1.0

    return sign * math.pow(abs(value), exp)

def near_zero(value: float) -> float:
    if abs(value) < 0.005:
        return 0.0
    else:
        return value


v1 = 0.0
v2 = 0.0
h0 = 0.5
h1 = 0.3
h2 = 0.2
def fir(v0: float) -> float:
    global v1
    global v2
    global h1
    global h2

    out = (v0 * h0 +
           v1 * h1 +
           v2 * h2)
    v2 = v1
    v1 = v0
    return out

def run_gamepad() -> None:
    """Poll the controller and send updates to robot object"""

    global rr

    # body rotations
    body_pitch = 0.0
    body_roll = 0.0
    body_yaw = 0.0
    angle_limit = math.radians(20.0)

    # Map controls
    axis_roll  = 'LEFT-X'
    axis_pitch = 'LEFT-Y'

    gamepadType = Gamepad.Xbox360
    pollInterval = 0.1

    if not Gamepad.available():
        print('Please connect your gamepad...')
        while not Gamepad.available():
            time.sleep(1.0)
    gamepad = gamepadType()
    print('Gamepad connected')

    # Start the background updating
    gamepad.startBackgroundUpdates()
    time.sleep(0.1)

    rr.stand()

    # Find the zero points
    roll_zero  = 0.0
    pitch_zero = 0.0
    iterations = 20
    for i in range(iterations):
        roll_zero  += gamepad.axis(axis_roll)
        pitch_zero += gamepad.axis(axis_pitch)
        time.sleep(0.05)
    roll_zero  /= float(iterations)
    pitch_zero /= float(iterations)

    # Joystick events handled in the background
    try:
        while gamepad.isConnected():

            # Update the joystick positions
            # body_roll  = angle_limit * near_zero(map_pow(fir(gamepad.axis(axis_roll)  - roll_zero)))
            # body_pitch = angle_limit * near_zero(map_pow(fir(gamepad.axis(axis_pitch) - pitch_zero)))
            body_roll  = angle_limit * near_zero(gamepad.axis(axis_roll)  - roll_zero)
            body_pitch = angle_limit * near_zero(gamepad.axis(axis_pitch) - pitch_zero)
            body_yaw   = 0.0
            rr.set_body_angles(body_roll, body_pitch, body_yaw)

            # Sleep for our polling interval
            time.sleep(pollInterval)
    finally:
        # Ensure the background thread is always terminated when we are done
        gamepad.disconnect()


if __name__ == "__main__":
    rr = mqtt_remote.Robot_MQTT()
    run_gamepad()
"""Provide an MQTT interface to do actions requested by remote user"""

"""
The messages use are.

robot/walk                  vel_x, vel_y, rot_z
robot/stand                 (None)
robot/set_velocity          vel_x, vel_y, rot_z
robot/set_body_angles       roll, pitch, yaw
robot/set_body_center       forward, left, up
"""

import robot
import json
import paho.mqtt.client as mqtt


class Robot_MQTT_Server:

    def __init__(self, r: robot.Robot):
        self.r = r

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.connect("spot")

    def run(self) -> None:
        self.client.loop_forever()

    def on_connect(self, client, userdata, flags, rc):
        """callback for when the client receives a CONNACK response from the server."""

        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code ", rc)
            return

        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        client.subscribe('robot_action/#')

    def on_message(self, client, userdata, msg):
        """callback for when a PUBLISH message is received from the server."""
        # print('Got message ', msg.topic, msg.payload)

        if len(msg.payload) == 0:
            data = ''
        else:
            data = json.loads(msg.payload)
        slash_indx = msg.topic.rfind('/')
        funtion = msg.topic[slash_indx+1:]

        if funtion == 'walk':
            self.r.walk(data[0], data[1], data[2])

        elif funtion == 'stand':
            self.r.stand()

        elif funtion == 'set_velocity':
            self.r.set_velocity(data[0], data[1], data[2])

        elif funtion == 'set_body_angles':
            angles = (data[0], data[1], data[2])
            self.r.set_body_angles(angles)

        elif funtion == 'set_body_center':
            self.r.set_body_center((data[0], data[1], data[2]))

        elif funtion == 'kill':
            self.r.kill()
            self.client.loop_stop()
            exit(0)

        else:
            print('Warning, unknown topic:', funtion)


if __name__ == "__main__":
    r = robot.Robot()
    rs = Robot_MQTT_Server(r)

    print('Calling rs.run()')
    rs.run()
    print('test complete.')

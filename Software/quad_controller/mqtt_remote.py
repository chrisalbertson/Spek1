"""Use MQTT interface to send requests to the robot"""

"""
The messages use are.

robot/walk                  vel_x, vel_y, rot_z
robot/stand                 (None)
robot/set_velocity          vel_x, vel_y, rot_z
robot/set_body_angles       roll, pitch, yaw
robot/set_body_center       forward, left, up

"""
import time
import json
#import robot
import paho.mqtt.client as mqtt


class Robot_MQTT:
    """An MQTT layer written around the robot class for remote operation"""
    def __init__(self):
        """Class Constructor"""

        self.client = mqtt.Client()
        self.client.on_connect = self.__on_connect
        self.client.on_message = self.__on_message

        self.client.connect("spot")

    def walk(self, vel_x: float = 0.0, vel_y: float = 0.0, rot_z: float = 0.0):
        """Publish a robot/walk message."""
        self.client.publish('robot_action/walk', json.dumps((vel_x, vel_y, rot_z)))

    def stand(self):
        """Publish a robot/stand message."""
        self.client.publish('robot_action/stand', '')

    def set_velocity(self, vel_x: float = 0.0, vel_y: float = 0.0, rot_z: float = 0.0):
        """Publish a robot/set_velocity message."""
        self.client.publish('robot_action/set_velocity', json.dumps((vel_x, vel_y, rot_z)))

    def set_body_angles(self, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0):
        """Publish a robot/set_body_angles message."""
        print('publish robot_action/set_body_angles', roll, pitch, yaw)
        self.client.publish('robot_action/set_body_angles', json.dumps((roll, pitch, yaw)))

    def set_body_center(self, forward: float = 0.0, left: float = 0.0, up: float = 0.0):
        """Publish a robot/set_body_center message."""
        self.client.publish('robot_action/set_body_center', json.dumps((forward, left, up)))

    def kill(self):
        """Publish a robot/kill message."""
        self.client.publish('robot_action/kill', '')

    def __on_connect(self, client, userdata, flags, rc):
        """callback for when the client receives a CONNACK response from the server."""

        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)
            return

        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        self.client.subscribe('robot_status/#')


    def __on_message(self, client, userdata, msg):
        """callback for when a PUBLISH message is received from the server."""
        print(msg.topic + " " + str(msg.payload))


if __name__ == "__main__":
    rr = Robot_MQTT()

    print('publishing walk')
    rr.walk(0.01, 0.0, 0.0)
    time.sleep(1.0)

    print('publishing set_velocity')
    rr.set_velocity(0.02, 0.0, 0.0)
    time.sleep(10.0)

    rr.kill()
    exit(0)


#!/usr/bin/python3

## Mnnual controller

from pynput.keyboard import KeyCode
from raceon.msg import AckermannDrive

class Controller:

    def __init__(self, default_speed=150, default_steering=800):
        self.default_speed = default_speed
        self.default_steering = default_steering

        self.speed = 0
        self.steering = 0
        self.steering_direction = 0
        self.terminate = False

    def on_press(self, key):
        if key == KeyCode.from_char('u'):
            self.speed = self.default_speed
        elif key == KeyCode.from_char('j'):
            self.speed = -self.default_speed
        elif key == KeyCode.from_char('h'):
            self.steering_direction = -1
        elif key == KeyCode.from_char('k'):
            self.steering_direction = 1
        elif key == KeyCode.from_char('a'):
            self.default_speed += 5
        elif key == KeyCode.from_char('z'):
            self.default_speed -= 5
        elif key == KeyCode.from_char('c'):
            if self.default_steering < 1000:
                self.default_steering += 100
        elif key == KeyCode.from_char('x'):
            if self.default_steering > 0:
                self.default_steering -= 100

    def on_release(self, key):
        if key == KeyCode.from_char('u') or key == KeyCode.from_char('j'):
            self.speed = 0
        elif key == KeyCode.from_char('h') or key == KeyCode.from_char('k'):
            self.steering_direction = 0
        elif key == KeyCode.from_char('q'):
            self.terminate = True

    def get_msg(self):
        self.steering = self.default_steering * self.steering_direction
        control_msg = AckermannDrive()
        control_msg.speed = self.speed
        control_msg.steering_angle = self.steering
        return control_msg

    def get_default_speed(self):
        return self.default_speed

    def get_default_steering(self):
        return self.default_steering

    def set_default_speed(self, val):
        self.default_speed = val

    def set_default_steering(self, val):
        self.default_steering = val

    def is_terminated(self):
        return self.terminate

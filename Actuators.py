# -*- coding: utf-8 -*-
"""
Created on Sat Sep  1 19:43:05 2018

@author: BrianPinto
"""
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM

class Valve(object):
    """ Software Representation of the Proportional Pressure Valve
    """

    def __init__(self, name, pwm_pin):
        """*Initialize with*

        Args:
           pwm_pin_0 (str): Pin for pwm 1, e.g. "P9_14"
        """
        self.name = name
        self.pwm_pin = pwm_pin

#        print(
#            'starting PWM with duty cycle 1. at Prportional Valve ', self.name)
        PWM.start(self.pwm_pin, 0, 25000)
        PWM.set_duty_cycle(self.pwm_pin, 10.0)

    def cleanup(self):
        """Stop pwm services."""
        print(
            'stop PWM duty cycle 0 Prportional Valve ', self.name)

        PWM.stop(self.pwm_pin)
        PWM.cleanup()

    def set_pwm(self, duty_cycle):
        """Set the pwm to **duty_cycle**

        Args:
            duty_cycle (int): Value between 0 to 100
        """
        PWM.set_duty_cycle(self.pwm_pin, duty_cycle)


class DiscreteValve(object):
    """ Software Representation of the Discrete Pressure Valve
    """

    def __init__(self, name, pin):
        """*Initialize with*

        Args:
           pin (str): Pin GPIO of BBB, e.g. "P8_7"
        """
        self.name = name
        self.pin = pin
        self.state = 0
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.output(self.pin, GPIO.LOW)

    def set_state(self, state):
        if self.state != state:
            if int(state) == 1:
                GPIO.output(self.pin, GPIO.HIGH)
                self.state = state
            elif int(state) == 0:
                GPIO.output(self.pin, GPIO.LOW)
                self.state = state

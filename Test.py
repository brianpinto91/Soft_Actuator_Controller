# -*- coding: utf-8 -*-
"""
Created on Thu Aug 15 22:26:35 2018

@author: BrianPinto
"""

#Imports
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_GPIO.I2C as Adafruit_I2C
import Adafruit_BBIO.PWM as PWM
#from __future__ import print_function
import time
import logging
import numpy as np


logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
formatter=logging.Formatter("%(asctime)s,%(message)s")
file_handler =logging.FileHandler('LogFile.log')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

console_handler = logging.StreamHandler()
console_handler.setFormatter(formatter)


#test belly top left
#pressure sensor connected on mplx_id 4
#MPU9150 sensor 0 connected on mplx_id 0
#MPU9150 sensor 1 connected on mplx_id 1
#Valve connected to PWM channel P9_22
#DiscreteValve connected to channel P8_10


#Define the ports
pValve0 = "P9_22"
dValve0 = "P8_10"
mplx_id_0 = 0
mplx_id_1 = 1
P_mplx_id = 4
stopButton = "P9_23"

def main():
    try:
        PWM = 10
        logger.debug("PWM is set to:{}".format(PWM))
        pSens0, IMUsens0, IMUsens1, pActuator, dActuator = initHardware()
        pActuator.set_pwm(PWM)
        while not GPIO.event_detected(stopButton):
            angle = calc_angle(IMUsens0,IMUsens1,-90)
            logger.debug("Pressure,{}, Angle,{}".format(pSens0.get_value(),angle))
            time.sleep(0.01)
    except Exception:
        logger.error("Error running the program")
    finally:
        GPIO.cleanup()
        PActuator.set_pwm(0)

def initHardware():
    pSens0 = DPressureSens(0,P_mplx_id)
    IMUsens0 = MPU_9150(0,mplx_id_0)
    IMUsens1 = MPU_9150(0,mplx_id_1)
    pActuator = Valve(0,pValve0)
    dActuator = DiscreteValve(0,dValve0)
    GPIO.setup(stopButton, GPIO.IN)
    GPIO.add_event_detect(stopButton, GPIO.RISING)
    #exitButton = stopButton
    return pSens0, IMUsens0, IMUsens1, pActuator, dActuator


class MultiPlexer(object):
    def __init__(self, address=0x70):
        self.i2c = Adafruit_I2C.get_i2c_device(address, busnum=2)

    def select(self, port_id):
        self.i2c.write8(0, 1 << port_id)
        

class DPressureSens(object):
    plexer = MultiPlexer()

    def __init__(self, name, mplx_id, address=0x28, maxpressure=1):
        self.mplx_id = mplx_id
        self.i2c = Adafruit_I2C.get_i2c_device(address, busnum=2)
        self.name = name
        self.maxpressure = maxpressure

        self.pmin = 0.0
        self.pmax = 150.0
        self.outmin = int((2**14-1)*.1)
        self.outmax = int((2**14-1)*.9)
        self.clb = (self.pmax-self.pmin)/(self.outmax - self.outmin)
        self.barfact = 1/14.5038

    def calc_pressure(self, msb, lsb):
        output = msb*256 + lsb
        pressure = ((output-self.outmin)*self.clb + self.pmin)*self.barfact
        return pressure

    def get_value(self):
        self.plexer.select(self.mplx_id)
        sens_bytes = self.i2c.readList(register=0, length=2)
        msb = sens_bytes[0]
        lsb = sens_bytes[1]
        pressure = self.calc_pressure(msb, lsb)
        return pressure/self.maxpressure

    def set_maxpressure(self, maxpressure):
        self.maxpressure = maxpressure
        
class MPU_9150(object):
    plexer = MultiPlexer(address=0x71)

    def __init__(self, name, mplx_id, address=0x68):
        power_mgmt_1 = 0x6b     # register power management of IMU
        self.name = name
        self.mplx_id = mplx_id    # plex ID of IMU
        # MultiPlexer schlaten, um das Modul ansprechen zu koennen
        self.i2c = Adafruit_I2C.get_i2c_device(address, busnum=2)
        self.plexer.select(self.mplx_id)
        time.sleep(.1)
        # Power on of Acc
        self.i2c.write8(power_mgmt_1, 0x00)

    def _read_word(self, reg):
        sens_bytes = self.i2c.readList(register=reg, length=2)
        msb = sens_bytes[0]
        lsb = sens_bytes[1]
        value = (msb << 8) + lsb
        return value

    def _read_word_2c(self, reg):
        val = self._read_word(reg)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def get_acceleration(self):
        self.plexer.select(self.mplx_id)

        acc_xout = self._read_word_2c(0x3b)
        acc_yout = self._read_word_2c(0x3d)
        acc_zout = self._read_word_2c(0x3f)
        return (acc_xout, acc_yout, acc_zout)
    
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
        PWM.set_duty_cycle(self.pwm_pin, 0)

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


def calc_angle(IMU1, IMU2, rotate_angle=0., delta_out=False):
    acc1=IMU1.get_acceleration()
    acc2=IMU2.get_acceleration()
    
    theta = np.radians(rotate_angle)
    acc1 = rotate(acc1, theta)
    x1, y1, z1 = normalize(acc1)
    x2, y2, z2 = normalize(acc2)
    phi1 = np.arctan2(y1, x1)
    vec2 = rotate([x2, y2, 0], -phi1+np.pi*.5)
    phi2 = np.degrees(np.arctan2(vec2[1], vec2[0]))

    alpha_IMU = -phi2+90

    if delta_out:
        z = np.mean([z1, z2])
        delta = np.degrees(np.arccos(z))

    return alpha_IMU if not delta_out else (alpha_IMU, delta)


def normalize(vec):
    x, y, z = vec
    l = np.sqrt(x**2 + y**2 + z**2)
    return x/l, y/l, z/l


def rotate(vec, theta):
    c, s = np.cos(theta), np.sin(theta)
    return (c*vec[0]-s*vec[1], s*vec[0]+c*vec[1], vec[2])  



if __name__ == "__main__":
    main()





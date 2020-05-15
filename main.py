# -*- coding: utf-8 -*-
"""
Created on Thu Aug 15 22:26:35 2018
@author: BrianPinto
"""

#Imports
import Adafruit_BBIO.GPIO as GPIO


import Sensors as Sensors
import Actuators as Actuators
import Controller as Controller
import time
import logging
import numpy as np
import datetime

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
formatter=logging.Formatter("%(message)s")
file_handler =logging.FileHandler('ContollerDebug.log')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

console_handler = logging.StreamHandler()
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)

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
sButton = "P9_23"

#Controller parameters
MAX_PWM = 75.0    # [10V]
MIN_PWM = 0.0
TSAMPLING = 0.001     # [sec]
PIDp = [1.05, 0.03, 0.01]    # [1]
PIDa = [0.0117, 1.012, 0.31]
MAX_PRESSURE = 1.0
MIN_PRESSURE = 0.


'''

'''
def main():
    pSens0, IMUsens0, IMUsens1, pActuator, dActuator, stopButton = initHardware()
    pController = Controller.PidController(PIDp,TSAMPLING,MAX_PWM,MIN_PWM)
    aController = Controller.PidController(PIDa,TSAMPLING,MAX_PRESSURE,MIN_PRESSURE)
   
    pActuator.set_pwm(10)
    time.sleep(5)
    startTime=datetime.datetime.now()
    
    """
            +                                              Pref    +                                  PWM
    Aref-----[ ]-----> aController ----> PressureSaturate --------> [ ]---> PController ---> SysInput ---> Actuator ---------------> Aout
            - ^                                                    - ^                                             |              | 
              |                                                      |                                             |---> Pout     |
              |                                                      -----------------------------------------------              |
              |                                                                                                                   |
              ---------------------------------------------------------------------------------------------------------------------
    """
    
    Aref = 40.0
    try:
       while not stopButton.isPressed():
           #read system output
           Pout = pSens0.get_value()
           Aout = calc_angle(IMUsens1,IMUsens0,0)
           logger.debug("Error, {}".format(Aref-Aout))
           #set pressure reference based on angle output
           #Bound the reference pressure between 0 and MAX_Pressure
           Pref = aController.output(Aref,Aout)
           logger.debug("Pref, {}".format(Pref))
           #Use pressure controller to follow the pressure reference
           ctr_out = pController.output(Pref,Pout)
           logger.debug("PWM, {}".format(ctr_out))
           pActuator.set_pwm(ctr_out)
           
           time.sleep(TSAMPLING)
           logReadings(IMUsens1,IMUsens0,pSens0,startTime,Aref)
    except Exception as err:
        logger.error("Error running the program: {}".format(err))
    finally:
        GPIO.cleanup()
        pActuator.set_pwm(10.0)
        
def logReadings(IMUsens1,IMUsens0,pSens0,startTime,Aref):
    angle = calc_angle(IMUsens1,IMUsens0,0)
    timeElapsed = datetime.datetime.now()-startTime
    logger.debug("Elapsed time, {}, ref, {}, presseure, {}, Angle, {}".format(timeElapsed,Aref,pSens0.get_value(),angle))        

def initHardware():
    pSens0 = Sensors.DPressureSens(0,P_mplx_id)
    IMUsens0 = Sensors.MPU_9150(0,mplx_id_0)
    IMUsens1 = Sensors.MPU_9150(0,mplx_id_1)
    pActuator = Actuators.Valve(0,pValve0)
    dActuator = Actuators.DiscreteValve(0,dValve0)
    stopButton = Sensors.Button(sButton)
    return pSens0, IMUsens0, IMUsens1, pActuator, dActuator, stopButton


def angleToPress(angle):
    return angle/90.0


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

# -*- coding: utf-8 -*-
"""
Created on Sat Sep  1 19:47:20 2018

@author: BrianPinto
"""

import abc
import numpy as np  # pragma: no cover


# pylint: disable=too-few-public-methods
class Controller(object):  # pragma: no cover
    """Base class for controllers. This defines the interface to controllers"""
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def reset_state(self):
        """
        Reset the states of the controller to zero
        """

    @abc.abstractmethod
    def set_maxoutput(self):
        """
        Set the maximal output of the Controller
        """

    @abc.abstractmethod
    def output(self, reference, system_output):
        """ compute the controller output and return

        Args:
            reference (float): where the system should be
            system_output (float): where the system actually is

        Returns:
            (float): controller_output for given input data
        """
        return


class PidController(Controller):
    """
    A simple PID controller
    """
    def __init__(self, gain, tsampling, max_output):
        """
        Args:
            gain (list): gains of the diffrent parts of ctr

                ====== = ============
                | gain = [Kp, Ti, Td]
                ====== = ============

            Ts (float): sampling time of the controller / overall system
            max_output (float): saturation of output

        Example:
            >>> Kp, Ti, Td = 10, 1.2, .4
            >>> gain = [Kp, Ti, Td]
            >>> tsampling, max_output = .001, 100
            >>> controller = PidController(gain, tsampling, max_output)
        """
        # Tuning Knobes
        self.Kp = gain[0]
        self.Ti = gain[1]
        self.Td = gain[2]
        self.max_output = max_output
        self.integral = 0.
        self.last_err = 0.
        self.last_out = 0.
        self.windup_guard = 0
        self.gam = .1   # pole for stability. Typically = .1
        self.tsampling = tsampling
        self.initial_cable_length = None

    def set_maxoutput(self, maxoutput):
        self.max_output = maxoutput

    def set_initial_cable_length(self, initial_cable_length):
        self.initial_cable_length = initial_cable_length

    def reset_state(self):
        self.integral = 0.
        self.last_err = 0.
        self.windup_guard = 0.
        self.last_out = 0.

    def set_gain(self, gain):
        self.Kp = gain[0]
        self.Ti = gain[1]
        self.Td = gain[2]
        self.reset_state()

    def output(self, reference, system_output):
        """
        The controller output is calculated by:

        ========== = ======================================
        | e        = r-y
        |
        | ctr_out  = Kp*e + Kp/Ti*integral(e) + Kp*Td*de/dt
        ========== = ======================================

        Args:
            reference (float): where the system should be
            system_output (float): where the system actually is

        Returns:
            (float): controller_output
        """
        # calc error
        err = reference - system_output
        # Derivative Anteil
        # diff = (err - self.last_err)/self.tsampling
        diff = (self.gam*self.Td - self.tsampling/2) / \
            (self.gam*self.Td + self.tsampling/2) * \
            self.last_out + \
            self.Td/(self.gam+self.tsampling/2)*(err-self.last_err)
        self.last_err = err
        # Integral Anteil
        integ = self.integral + self.tsampling / \
            (2*self.Ti)*(err-self.windup_guard)
        if np.abs(integ) > self.max_output:
            integ = self.max_output*np.sign(integ)
        self.integral = integ

        # Sum
        controller_output = self.Kp*(err + integ + diff)

        if np.abs(controller_output) > self.max_output:
            self.windup_guard = controller_output * \
                (1-self.max_output/abs(controller_output))
            self.last_out = self.max_output*np.sign(controller_output)
        else:
            self.windup_guard = 0
            self.last_out = controller_output
        return self.last_out

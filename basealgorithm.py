#!/usr/bin/env python
 
from abc import ABCMeta, abstractmethod
import logging

class BaseState(object):
    __metaclass__ = ABCMeta

    def __init__(self, log_level=logging.INFO):
        logging.basicConfig(level=log_level)
        self.logger = logging.getLogger(__name__)

    @abstractmethod
    def get_cost(self):
        pass

    @abstractmethod
    def print_state(self):
        pass

    @abstractmethod
    def compare(self, state1):
        """
        return: can be any string which meaning is defined by specific problem
        """
        pass


class BaseManipulator(object):
    __metaclass__ = ABCMeta

    def __init__(self, log_level=logging.INFO):
        logging.basicConfig(level=log_level)
        self.logger = logging.getLogger(__name__)

    @abstractmethod
    def randomize_state(self):
        pass
    
    @abstractmethod
    def generate_state(self, state):
        """
        state: based on this state, newly close states will be generated
        """
        pass


class BaseAlgorithm(object):
    __metaclass__ = ABCMeta

    def __init__(self, log_level=logging.INFO):
        logging.basicConfig(level=log_level)
        self.logger = logging.getLogger(__name__)
        self._cur_state = None
        self.manipulator = None

    def get_current_state(self):
        return self._cur_state

    @abstractmethod
    def stop_condition(self):
        pass

    @abstractmethod
    def execute(self):
        pass

#!/usr/bin/env python

from abc import ABCMeta, abstractmethod
import numpy as np
import logging
import base.basealgorithm as balg

class TabuSearchManipulator(balg.BaseManipulator):
    __metaclass__ = ABCMeta
    
    @abstractmethod
    def generate_neighbor_states(self, state):
        "return a candidate list with a corresponding 'moves' list "
        pass

class TabuSearchAlgorithm(balg.BaseAlgorithm):
    """
    _cur_state: current best candidate
    _best_state: current best solution
    """
    def __init__(self, manipulator, maxTabuSize, log_level=logging.INFO):
        super(TabuSearchAlgorithm, self).__init__(log_level)
        self.manipulator = manipulator
        self._best_state = None
        self._tabu_list = []
        self._max_TabuSize = maxTabuSize


    def execute(self):
        self._cur_state = self.manipulator.randomize_state()
        self._best_state = self._cur_state
        while self.stop_condition() != True:
            candidates, moves = self.manipulator.generate_neighbor_states(self._cur_state)

            for c,m in zip(candidates, moves):
                if m not in self._tabu_list and c.get_cost() < self._cur_state.get_cost():
                    self._cur_state = c

            if self._cur_state.get_cost() < self._best_state.get_cost():
                self._best_state = self._cur_state

            self._tabu_list.append(self._best_state)

            if len(self._tabu_list) > self._max_TabuSize:
                self._tabu_list.pop(0)
            

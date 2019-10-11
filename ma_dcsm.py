import numpy as np
import numpy.random as npr
import matplotlib.pyplot as plt
import markovapproxbase as mab
import logging
import json

class DCSM(mab.MarkovApproxBase):

    def __init__(self, nbr_sw, nbr_controller, E, c_cap, cc, sd, f, log_level=logging.INFO):
        """
        :param M: number of switches
        :param N: number of controllers
        :param E: an N-element array of controllers' energy
        :param c_cap: an N-element array of controllers' capacity
        :param cc: an NxN matrix of the cost between controllers
        :param sd: an M-element array of demands of each switch
        :param f: an MxM matrix, relationship between switches, representing the flow of traffic going through switches
        """
        super(DCSM, self).__init__(log_level)
        self.nbr_switches = nbr_sw
        self.nbr_controllers = nbr_controller
        self.controllers_energy = E
        self.controllers_cap = c_cap
        self.cc_cost = cc
        self.sw_demand = sd
        self.switches_flow = f

        self.stop_cond = 0
        self.costs = []

        self.delta = 0.03

        # shared_info stores id of switch that change connection between 2 controllers
        self.shared_info = {'cost_diff': 0, 'state': {'selected_sw': 0, 'selected_ctrl': 0, 'last_selected_ctrl': 0}}

    def initialize_state(self, support_info=None):
        # state, representing the assignment between a switch and a controller
        initial_state = np.zeros((self.nbr_switches, self.nbr_controllers))
        dec_idx = reversed(np.argsort(self.controllers_cap))
        sw_idx = 0  # switch index
        for ctrl_idx in dec_idx:  # ctrl_idx: controller index, starts from the most powerful controller
            c = 0
            while c < self.controllers_cap[ctrl_idx] and sw_idx < self.nbr_switches:
                if (c + self.sw_demand[sw_idx]) > self.controllers_cap[ctrl_idx]:
                    break
                initial_state[sw_idx, ctrl_idx] = 1
                c = c + self.sw_demand[sw_idx]
                sw_idx = sw_idx + 1
        return initial_state

    def generate_next_state(self, cur_state):
        nxt_state = np.copy(cur_state)
        selected_sw_id = npr.random_integers(1, self.nbr_switches) - 1
        selected_ctrl_id = npr.random_integers(1, self.nbr_controllers) - 1
        last_selected_ctrl_id = list(nxt_state[selected_sw_id, :]).index(1)
        nxt_state[selected_sw_id, :] = np.zeros(self.nbr_controllers)
        nxt_state[selected_sw_id, selected_ctrl_id] = 1
        self.shared_info['state'] = {'selected_sw': selected_sw_id, 'selected_ctrl': selected_ctrl_id, 'last_selected_ctrl': last_selected_ctrl_id}
        return nxt_state

    def state_cost(self, state):
        te = 0
        for i in range(self.nbr_switches):
            for j in range(self.nbr_controllers):
                te = te + state[i, j] * self.sw_demand[i] * self.controllers_energy[j] / (1.0 * self.controllers_cap[j])
        tc = 0
        for i in range(self.nbr_switches):
            for j in range(self.nbr_switches):
                for k in range(self.nbr_controllers):
                    for l in range(self.nbr_controllers):
                        tc = tc + self.switches_flow[i, j] * state[i, k] * state[j, l] * self.cc_cost[k, l]
        return te + tc / 2.0

    def cost_diff(self, state1, state2):
        """assumption: only one switch that changes its connection between two controllers in two states
        :param state1: current state
        :param state2: next state
        :return: cost(state2) - cost(state1)
        """
        selected_sw_id = self.shared_info['state']['selected_sw']
        selected_ctrl_id = self.shared_info['state']['selected_ctrl']
        last_selected_ctrl_id = self.shared_info['state']['last_selected_ctrl']
        delta_cost = 0
        for i in range(self.nbr_switches):
            for j in range(self.nbr_controllers):
                delta_cost = delta_cost + self.switches_flow[selected_sw_id, i] * \
                             (state2[i, j] * self.cc_cost[selected_ctrl_id, j] - state1[i, j]
                              * self.cc_cost[last_selected_ctrl_id, j])
        delta_cost = delta_cost + self.sw_demand[selected_sw_id] * (self.controllers_energy[selected_ctrl_id] / (1.0 * self.controllers_cap[selected_ctrl_id]) - self.controllers_energy[last_selected_ctrl_id] / (1.0 * self.controllers_cap[last_selected_ctrl_id]))
        self.shared_info['cost_diff'] = delta_cost
        return delta_cost

    def transition_rate(self, cur_state, next_state):
        rate = np.exp(-0.1) / (1.0 + np.exp(-self.delta * -self.cost_diff(cur_state, next_state)))
        self.logger.debug("Cost diff: %f, Transition rate: %f", self.shared_info['cost_diff'], rate)
        return rate

    def stop_condition(self):
        self.stop_cond = self.stop_cond + 1
        if self.stop_cond <= 1000:
            return True
        return False

    def execute(self):
        initial_state = self.initialize_state()
        print initial_state
        cur_state = np.copy(initial_state)
        cur_cost = self.state_cost(cur_state)
        self.costs.append(cur_cost)
        while self.stop_condition() is True:
            next_state = self.generate_next_state(cur_state)
            required_cap = np.dot(next_state[:, self.shared_info['state']['selected_ctrl']], np.reshape(self.sw_demand, (self.nbr_switches, 1)))
            if (required_cap > self.controllers_cap[self.shared_info['state']['selected_ctrl']]) \
                    or (self.transition_condition(cur_state, next_state) is False):
                continue
            next_cost = cur_cost + self.shared_info['cost_diff']
            self.logger.debug('Current cost: %f, next cost: %f', cur_cost, next_cost)
            self.costs.append(next_cost)
            cur_cost = next_cost
            cur_state = np.copy(next_state)

    def plot_convergence(self):
        for c in self.costs:
            print(c)


        plt.interactive(False)
        plt.plot(self.costs)
        plt.show()


if __name__ == '__main__':
    nbr_sw = 20  # nbr of switches
    nbr_controller = 11  # nbr of controllers
    E = npr.randint(20, 30, nbr_controller)  # unit energy consumption
    c_cap = npr.randint(30, 100, nbr_controller)  # capacity of a controller
    sd = npr.randint(5, 30, nbr_sw)  # demand from switch to controller
    f = npr.uniform(0, 5, (nbr_sw, nbr_sw))  # flow link between switch and switch
    #f = 5*npr.randint(0, 2, (nbr_sw, nbr_sw))  # flow link between switch and switch
    cc = npr.randint(1, 20, (nbr_controller, nbr_controller))  # capacity between controller and controller
    for i in range(nbr_controller):
        cc[i, i] = 0
        for j in range(nbr_controller):
            cc[i, j] = cc[j, i]

    for i in range(nbr_sw):
        f[i, i] = 1
        for j in range(nbr_sw):
            f[i, j] = f[j, i]
    """ a simple case to test
    M = 5  # nbr of switches
    N = 4  # nbr of controllers
    cc = np.array([[0, 5, 8, 3],
                  [5, 0, 1, 7],
                  [8, 1, 0, 9],
                  [3, 7, 9, 0]])
    f = np.array([[1, 0, 1, 0, 0],
                  [0, 1, 0, 1, 0],
                  [1, 0, 1, 0, 1],
                  [0, 1, 0, 1, 0],
                  [0, 0, 1, 0, 1]])
    c_cap = np.array([30, 20, 35, 45])
    sd = np.array([13, 12, 10, 15, 20])
    """

    dcsm = DCSM(nbr_sw, nbr_controller, E, c_cap, cc, sd, f, logging.DEBUG)
    dcsm.execute()
    dcsm.plot_convergence()

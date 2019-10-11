import numpy as np
import numpy.random as npr
import matplotlib.pyplot as plt
import ma
import logging
import json
import itertools
import pickle
import threading

class MAVP(ma.MarkovApproxBase):

    def __init__(self, log_level=logging.INFO):
        """
        :param M: number of switches
        :param N: number of controllers
        :param E: an N-element array of controllers' energy
        :param c_cap: an N-element array of controllers' capacity
        :param cc: an NxN matrix of the cost between controllers
        :param sd: an M-element array of demands of each switch
        :param f: an MxM matrix, relationship between switches, representing the flow of traffic going through switches
        """
        super(MAVP, self).__init__(log_level)
        self._nbr_clouds = 3
        self._nbr_gateways = 5
        self._nbr_vnf = 10
        self._nbr_vnf_instances = np.ones(self._nbr_vnf)
        self._nbr_sc = 2
        self._sc_rates = [0.1,0.1]
        self._bw_sensing_vnf = npr.uniform(0.1, 0.7, self._nbr_vnf)
        self._bw_output_vnf = npr.uniform(1.5, 2.5, self._nbr_vnf)
        self._tho_vnf = np.ones(self._nbr_vnf)
        self._set_v_g = np.array([[0], [1, 2], [3], [4, 5, 6], [7, 8, 9]])
        self._r = npr.randint(2,5, self._nbr_clouds)
        self._beta = np.array([np.zeros((self._nbr_vnf, self._nbr_vnf)), np.zeros((self._nbr_vnf, self._nbr_vnf))])
        self._beta[0][0,2] = self._beta[0][2,4] = self._beta[0][4,6] = 1
        self._beta[1][1,3] = self._beta[1][3,5] = self._beta[1][5,7] = self._beta[1][7,8] = self._beta[1][8,9] = 1

        self.B_n_n = np.zeros(shape=(self._nbr_clouds, self._nbr_clouds))
        self.B_g_n = np.zeros(shape=(self._nbr_gateways, self._nbr_clouds))
        self.R_n = np.zeros(self._nbr_clouds)

        self._net_cost = np.zeros((self._nbr_clouds + self._nbr_gateways, self._nbr_clouds + self._nbr_gateways))
        for i,j in itertools.combinations(list(range(self._nbr_clouds + self._nbr_gateways)),r=2):
            self._net_cost[i,j] = npr.uniform(4.8, 5.2)

        self._com_cost = npr.uniform(2.8, 3.2, self._nbr_clouds)
        self.x = np.zeros(shape=(self._nbr_clouds, self._nbr_vnf))

        self.costs = []

        self.stop_cond = 0
        self.delta = 0.5

    def _get_objetive_cost(self, w):
        weighted_net_cost = np.sum(np.triu(self.B_n_n)*np.triu(self._net_cost[0:self._nbr_clouds, 0:self._nbr_clouds])) \
            + np.sum(self.B_g_n*self._net_cost[self._nbr_clouds:, 0:self._nbr_clouds])
        return w*weighted_net_cost + (1 - w)*np.sum(self.R_n*self._com_cost)

    def _get_constraint_1(self):
        self.B_g_n = np.zeros(shape=(self._nbr_gateways, self._nbr_clouds))
        for g in range(self._nbr_gateways):
            for n in range(self._nbr_clouds):
                tmp = np.take(self._bw_sensing_vnf, self._set_v_g[g])*self.x[n,self._set_v_g[g]]
                self.B_g_n[g,n] = np.sum(tmp)
        return self.B_g_n
    
    def _get_constraint_2(self):
        self.B_n_n = np.zeros(shape=(self._nbr_clouds, self._nbr_clouds))
        for n in range(self._nbr_clouds):
            for m in range(self._nbr_clouds):
                for c in range(self._nbr_sc):
                    for v in range(self._nbr_vnf):
                        self.B_n_n[n,m] = self.B_n_n[m,n] = self.B_n_n[n,m] + np.sum(self._beta[c][v,:]*self._bw_output_vnf[v]*self.x[n,v]*self.x[m,:])
        return self.B_n_n
    
    def _get_constraint_3(self):
        self.R_n = np.zeros(self._nbr_clouds)
        b = np.zeros(self._nbr_vnf)
        for u in range(self._nbr_vnf):
            for c in range(self._nbr_sc):
                b += self._beta[c][:,u]*self._bw_output_vnf[:] + self._tho_vnf[:]*self._bw_sensing_vnf[:]
        for n in range(self._nbr_clouds):
            self.R_n[n] = np.sum(b*self.x[n,:]*self._r[n])
        
        return self.R_n

    def initialize_state(self, support_info=None):
        is_ = np.zeros((self._nbr_clouds, self._nbr_vnf))
        is_[0,:] = np.ones(self._nbr_vnf)
        return is_


    def generate_next_state(self, cur_state):
        nxt_state = np.copy(cur_state)
        selected_vnf = npr.randint(0, self._nbr_vnf)
        selected_cloud = npr.randint(0, self._nbr_clouds)
        nxt_state[selected_cloud, selected_vnf] = abs(nxt_state[selected_cloud, selected_vnf] - 1) 
        vnf_placement = nxt_state[:, selected_vnf]
        if vnf_placement[vnf_placement == 1].size > self._nbr_vnf_instances[selected_vnf]:
            cloud_locations = np.where(vnf_placement == 1)[0]
            # un-place randomly selected VNF from a certain cloud
            nxt_state[npr.choice(cloud_locations), selected_vnf] = 0
        elif vnf_placement[vnf_placement == 1].size == 0:
            nxt_state[npr.randint(0, self._nbr_clouds), selected_vnf] = 1

        self.x = nxt_state
        return nxt_state

    def state_cost(self, state):
        self.x = np.copy(state)

        self._get_constraint_1()
        self._get_constraint_2()
        self._get_constraint_3()
        
        return self._get_objetive_cost(0.5)

    def transition_rate(self, cur_state, nxt_state):
        rate = np.exp(self.delta*(self.state_cost(cur_state) - self.state_cost(nxt_state)))
        print("[transition_rate] cur_state:\n",  cur_state, self.state_cost(cur_state))
        print("[transition_rate] nxt_state:\n",  nxt_state, self.state_cost(nxt_state))
        print("[transition_rate] rate:",  rate)
        return rate

    def stop_condition(self):
        self.stop_cond = self.stop_cond + 1
        if self.stop_cond <= 100:
            return True
        return False

    def execute(self):
        initial_state = self.initialize_state()
        #print initial_state
        cur_state = np.copy(initial_state)
        while self.stop_condition() is True:
            next_state = self.generate_next_state(cur_state)
            if (self.transition_condition(cur_state, next_state) is False):
                continue
            self.costs.append(self._get_objetive_cost(0.5))
            cur_state = np.copy(next_state)

    def _save_configuration(self):
        with open('mavp.dump', 'wb') as config_dictionary_file: 
            pickle.dump(self, config_dictionary_file)

    def plot_convergence(self):
        threading.Thread(target=self._save_configuration).start()

        plt.interactive(False)
        plt.plot(self.costs)
        plt.show()


if __name__ == '__main__':
    mavp = MAVP(logging.DEBUG)
    mavp.execute()
    mavp.plot_convergence()
    

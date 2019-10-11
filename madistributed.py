from abc import ABCMeta, abstractmethod
import thread
import numpy as np
import numpy.random as npr
import matplotlib.pyplot as plt
import mavp
import logging
import socket


class DMAController(ma.MarkovApproxBase):

    def __init__(self, log_level=logging.INFO):
        super(MarkovApproxDistributed, self).__init__(log_level)
        self._listening_ip = ip
        self._listening_port = p
        self.connected_workers = []
        self.nbr_workers = 5
        self.cur_state = 0
        self._states_from_clients = []
        self._max_nbr_states_receiving_from_clients = 10
        self.state_distance_threshold = 100


    def _clientHandler(con, addr):
        print(addr, "is Connected")
        try:
            while True:
                data = con.recv(1024)
                if not data: 
                    break
                self._batch_states.append(data)
                if(len(self._batch_states) == self._max_nbr_states_receiving_from_clients):
                    c = 10000
                    min_state_idx = 0
                    for i in range(self._max_nbr_states_receiving_from_clients):
                        _c = self.state_cost(self._states_from_clients[i])
                        if(c > _c):
                            min_state_idx = i
                            c = _c

                    for i in range(self._max_nbr_states_receiving_from_clients):
                        cd = abs(self.cost_diff(self._states_from_clients[i], self._states_from_clients[min_state_idx]))
                        if(cd < self.state_distance_threshold):
                            con.sendto(self.generate_next_state(self.cur_state), w)
                else:
                    for w in self.connected_workers:
                        if addr != w:
                            con.sendto(self.generate_next_state(self.cur_state), w)
        except:
            print("Error. Data not sent to all clients.")

    def start_listening(self):
        s = socket(AF_INET, SOCK_STREAM)
        s.bind((self._listening_ip, self._listening_port))

        print("Server is running on "+ str(self._listening_port))
        connection_threads = []
        for i in range(self.nbr_workers): 
            con, addr = s.accept() 
            clients.append(addr)
            t = Thread(target=self._clientHandler, args = (con, addr))
            connection_threads.append(t)
            t.start()

        for t in connection_threads:
            t.join()

        s.close()
    
    def execute(self):
        initial_state = self.initialize_state()


class DMAWorker(mavp.MAVP):

    def __init__(self, log_level=logging.INFO):
        super(MAVP, self).__init__(log_level)
        self._server_ip = ip
        self._server_port = p
        self._batch_size = 5

    def sending_state(self):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((self._server_ip, self._server_port))
        client_socket.send('hello')
        while True:
            cur_state = client_socket.recv(64)



#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Project ：UavNetSim
@File    ：qfanet_table.py
@Author  ：abing xbb992@vip.qq.com
@Date    ：2025/8/20
@Update  ：2025/8/29
'''
import math

import numpy as np
from collections import defaultdict

from utils import util_function
from utils.util_function import euclidean_distance_3d
from phy.large_scale_fading import maximum_communication_range


class QFanetTable:
    """
    Q-FANET tables
    """
    """
            The structure of the neighbor table in Q-FANET
            |    Drone id   |  1  |  2       |    3      |    4   |  5   |        6           |   7   |
            | neighbor_id 1 | pos | velocity | timestamp | energy | sinr | [dst_id:{rewards}] | delay |
            | neighbor_id 2 | pos | velocity | timestamp | energy | sinr | [dst_id:{rewards}] | delay |
            |       ...     | ... | ...      |
            | neighbor_id N | pos | velocity | timestamp | energy | sinr | [dst_id:{rewards}] | delay |
            """
    def __init__(self, env, my_drone, rng_routing):

        self.env = env
        self.my_drone = my_drone
        self.rng_routing = rng_routing
        self.neighbor_table = defaultdict(lambda: [None, None, 0, 0, [], defaultdict(list),None])
        self.entry_life_time = 2 * 1e6
        self.lookback = 10  # history window size

        n_drones = my_drone.simulator.n_drones
        self.q_table = np.full((n_drones, n_drones), 0.5)  # 初始值0.5
        self.max_comm_range = maximum_communication_range()

    def is_neighbor(self, drone_id):
        """Check if the drone is a neighbor"""
        if drone_id not in self.neighbor_table:
            return False
        # check if the neighbor entry is expired
        if self.neighbor_table[drone_id][2] + self.entry_life_time <= self.env.now:
            return False
        dist = euclidean_distance_3d(
            self.my_drone.coords,
            self.neighbor_table[drone_id][0]
        )
        return dist <= self.max_comm_range

    def add_neighbor(self, hello_packet, cur_time, cur_sinr):
        """Add neighbor to the table"""
        src_id = hello_packet.src_drone.identifier
        if src_id == self.my_drone.identifier:
            return
        delay = cur_time - hello_packet.creation_time

        self.neighbor_table[src_id][0] = hello_packet.cur_position
        self.neighbor_table[src_id][1] = hello_packet.cur_velocity
        self.neighbor_table[src_id][2] = cur_time
        self.neighbor_table[src_id][3] = hello_packet.src_energy
        self.neighbor_table[src_id][4] = cur_sinr
        self.neighbor_table[src_id][6] = delay
        # [pos,velocity,timestamp,energy,sinr,list of reward history,delay]


    def purge(self):
        """Remove expired neighbor entry"""
        for drone_id in list(self.neighbor_table.keys()):
            if not self.is_neighbor(drone_id):
                del self.neighbor_table[drone_id]

    def calculate_eta(self, sinr):
        """Transform SINR to eta value"""
        if sinr < 8:
            return 0.0
        elif 8 <= sinr < 10:
            return 0.25
        elif 10 <= sinr < 15:
            return 0.5
        elif 15 <= sinr < 17:
            return 0.75
        else:
            return 1.0

    def calculate_velocity_constraint(self, neighbor_id, dst_drone):
        """Calculate Velocity"""
        if not self.is_neighbor(neighbor_id):
            return -1

        d_myself_dst = euclidean_distance_3d(self.my_drone.coords, dst_drone.coords)
        d_neighbor_dst = euclidean_distance_3d(self.neighbor_table[neighbor_id][0], dst_drone.coords)

        delay = self.neighbor_table[neighbor_id][6] / 1e6
        return (d_myself_dst - d_neighbor_dst) / delay

    def void_area_judgment(self, dst_drone):
        """Routing hole judgment"""
        self.purge()
        if not self.neighbor_table:
            return 1

        d_myself_dst = euclidean_distance_3d(self.my_drone.coords, dst_drone.coords)
        for neighbor_id in self.neighbor_table:
            if not self.is_neighbor(neighbor_id):
                continue
            d_neighbor_dst = euclidean_distance_3d(self.neighbor_table[neighbor_id][0], dst_drone.coords)
            if d_neighbor_dst <= d_myself_dst:
                return 0
        return 1

    def get_candidate_neighbors(self, dst_drone, packet):
        """Obtain the candidates neighbors"""
        self.purge()
        packet_deadline = (packet.creation_time + packet.deadline - self.env.now) / 1e6
        # calculate the required velocity
        dist_to_dest = util_function.euclidean_distance_3d(self.my_drone.coords, dst_drone.coords)
        required_velocity = dist_to_dest / packet_deadline
        candidates = []
        sub_candidates = []
        for neighbor_id in self.neighbor_table:
            velocity = self.calculate_velocity_constraint(neighbor_id, dst_drone)
            if velocity > required_velocity:
                candidates.append(neighbor_id)
            elif velocity > 0:
                # velocity smaller than required_velocity but greater than 0
                sub_candidates.append((neighbor_id, velocity))

        return candidates, sub_candidates

    def best_neighbor(self, packet, dst_drone, epsilon=0.1):
        self.purge()
        dst_id = dst_drone.identifier
        if len(self.neighbor_table.keys()) == 0:
            return self.my_drone.identifier
        # candidates 满足约束的邻居
        # sub 仅仅大于0的邻居
        candidates, sub_candidates = self.get_candidate_neighbors(dst_drone, packet)
        if len(candidates) > 0:
            # Q-Noise+
            if self.rng_routing.random() > epsilon:
                best_id = self.rng_routing.choice(list(candidates))
            else:
                best_id = max(candidates, key=lambda x: self.q_table[x][dst_id])
        elif len(sub_candidates) > 0:
            best_id = max(sub_candidates, key=lambda x: x[1])[0]
        else:
            # routing hole
            best_id = self.my_drone.identifier
        return best_id

    def update_q_value(self, next_hop_id, dst_id, reward, sinr_eta):
        """Q-Noise+ update"""
        if not self.is_neighbor(next_hop_id):
            return
        # get the lookback rewards history
        lookback_rewards = self.neighbor_table[next_hop_id][5].get(dst_id, [])
        lookback_rewards = lookback_rewards[-self.lookback:]
        # generate random weights
        weights = [self.rng_routing.random() for _ in range(self.lookback)]
        weights = [w / sum(weights) for w in weights]

        while len(lookback_rewards) < self.lookback:
            lookback_rewards.insert(0, 0)
        weighted_historical = sum(w * r for w, r in zip(weights, lookback_rewards))

        self.q_table[next_hop_id][dst_id] = (1 - 0.6) * weighted_historical + \
                                            0.6 * reward + \
                                            (0.7 * sinr_eta)

        self.neighbor_table[next_hop_id][5][dst_id].append(reward)
        self.neighbor_table[next_hop_id][5][dst_id] = self.neighbor_table[next_hop_id][5][dst_id][-self.lookback:]



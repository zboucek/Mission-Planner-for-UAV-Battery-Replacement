# Credit for this: Nicholas Swift
# as found at https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
# modified to schedule drone battery swapping with numpy array as progress (SEQ of in mission array for i-th drone)
# Lines corresponds to pseudocoude
from warnings import warn
import heapq
import numpy as np
import copy
import time
import os, sys


class AStar(object):
    """A* algorithm for search of optimal path in graph."""

    def __init__(self, drones, ports, start, end, actions, actions_cost, actions_time, soc_thr=0.2, time_thr=60.0, max_iterations = 1e5):
        """ init A* algorithm

        input:
            maze: np array with maze
            start: start coordinates
            end: end coordinates
            diagonal: 'True' if diagonal movement is possible
        """
        # 1: function MAIN
        self.path = []
        self.start_time = 0
        self.end_time = np.inf

        # Initialize both open and closed list
        # 2: open = closed = {}
        self.open_list = []
        self.closed_list = []
        
        self.drones = drones
        self.ports = ports
        self.actions = actions
        self.actions_cost = actions_cost
        self.actions_time = actions_time
        self.soc_thr = soc_thr
        self.time_thr = time_thr
        
        self.max_iterations = max_iterations

        # Create start and end node
        # 4: parent(s_start) = s_start
        self.start_node = self.Node(None, progress=start.copy())
        # 3: g(s st art ) = 0
        self.start_node.g = self.start_node.h = self.start_node.f = 0
        self.start_node.parent = self.start_node
        self.start_node.n_batt = np.empty(len(self.ports), dtype=int)
        for port in self.ports:
            self.start_node.n_batt[port.id-201] = port.slots
        for drone in self.drones:
            self.start_node.missions_time.append(drone.time_plan.copy())
            self.start_node.soc_progress.append(
                drone.battery.current_capacity-np.cumsum(drone.mission_soc))
            self.start_node.progress[drone.id-1] = drone.mission_count-1
            for i, soc in enumerate(self.start_node.soc_progress[drone.id-1]):
                if soc < soc_thr:
                    if i == 0:
                        self.start_node.progress[drone.id-1] = 0
                    else:
                        self.start_node.progress[drone.id-1] = i-1
                    break

        self.end_node = self.Node(None, end.copy())
        self.end_node.g = self.end_node.h = self.end_node.f = 0

        self.end_node.g = np.inf
        self.end_node.f = self.end_node.g + self.end_node.h
        self.start_node.h = self.soc2end(self.start_node, self.drones)
        self.start_node.f = self.start_node.g + self.start_node.h
        # Heapify the open_list and Add the start node
        heapq.heapify(self.open_list)
        # 5: open.Insert(s_start, g(s_start) + h(s_start))
        heapq.heappush(self.open_list, self.start_node)

    class Node:
        """ A node class for A* Pathfinding
        """

        def __init__(self, parent=None, progress=None, soc_progress=[], n_batt=[], action=None, action_cost=None, action_time=None, missions_time=[], schedule=[]):
            self.parent = parent
            self.progress = progress
            self.soc_progress = soc_progress
            self.n_batt = n_batt
            self.action = action
            self.action_cost = action_cost
            self.action_time = action_time
            self.missions_time = missions_time
            self.schedule = schedule

            self.g = np.inf
            self.h = 0.0
            self.f = np.inf

        def __eq__(self, other):
            return self.progress == other.progress

        def __repr__(self):
            return f"{self.progress} - g: {self.g} h: {self.h} f: {self.f}"

        # defining less than for purposes of heap queue
        def __lt__(self, other):
            return self.f < other.f

        # defining greater than for purposes of heap queue
        def __gt__(self, other):
            return self.f > other.f


    def return_plan(self, current_node):
        """ Return path from origin to current_node.
        """
        path = []
        current = current_node
        while True:
            path.append(current)
            if current.parent is current:
                # start node is reached
                break
            current = current.parent
        return path[::-1]  # Return reversed path


    def soc2end(self, start, drones):
        """ Estimate of battery consumption until the end of missions 
        """
        if start is None:
            return np.nan
        else:
            soc = 0.0
            for drone in drones:
                soc = soc + \
                    np.sum(
                        np.abs(drone.mission_soc[start.progress[drone.id-1]+1:]))

            return soc


    def update_node(self, current, child, drones, open_list):
        """ Update node

        input:
            current: original goal
            child: goal node
            open_list: list with explored nodes
        """
        # 23: function UpdateVertex(s, s')
        # 24: g_old = g(s')
        # 25: ComputeCost(s, s')
        self.compute_cost(current, child, child.action_cost)
        child.h = self.soc2end(child, drones)
        child.f = child.g + child.h
        # 26: if g(s') < g_old then
        # if child.g < g_old:
        # 27: if s' in open then
        if any((child == x).all() for x in open_list):
            return
        # 30: open.Insert(s', g(s') + h(s'))
        heapq.heappush(open_list, child)


    def compute_cost(self, current, child, action_cost):
        """ Edit node according to cost.

        input:
            current: original goal
            child: goal node
        """
        # g(s') = g(s) + c(s, s')
        c = 2*action_cost   # travel to Droneport and back
        # Path 1
        # 34: if g(s) + c(s, s') < g(s') then
        if current.g + c < child.g:
            # 35: parent(s') = s
            child.parent = current
            # 35: g(s') = g(s) + c(s, s')
            child.g = current.g + c


    def plan_missions(self):
        """ Returns a list of tuples as a path from the given start to the given end in the given maze

        input:
            maze: np array with maze
            start: start coordinates
            end: end coordinates
        output:
            path OR None
        """
        
        # Adding a stop condition
        outer_iterations = 0

        # Loop until you find the end
        # 6: while open!={} do
        while len(self.open_list) > 0:
            outer_iterations += 1

            if outer_iterations > self.max_iterations:
                # if we hit this point return the path such as it is
                # it will not contain the destination
                warn("giving up on pathfinding too many iterations")
                return self.return_plan(current_node)

            # Get the current node
            # 7: s = open.Pop()
            current_node = heapq.heappop(self.open_list)
            # 8: [SetVertex(s)]
            # 12: closed = closed U {s}
            self.closed_list.append(current_node)

            # Found the goal
            # 9: if s = s_goal then
            if not False in (current_node.progress == self.end_node.progress):
                # 10: return “path found”
                return self.return_plan(current_node)

            # Generate children
            children = []

            # 13: forall s' in nghbr_vis(s) do
            for action in self.actions:  # Adjacent squares
                # action = [drone.id, port.id, k]
                if True in (action == -1):
                    continue
                
                action_cost = self.actions_cost[action[0]-1, action[1]-201, action[2]]
                action_time = self.actions_time[action[0]-1, action[1]-201, action[2]]

                # Make sure action is valid
                if action_cost == -1 or action_time == -1:
                    continue

                # check if Droneport is free
                time_at_waypoint = current_node.missions_time[action[0]-1][action[2], 0]
                time_at_droneport = time_at_waypoint + action_time
                time_after_swap = time_at_droneport + \
                    self.ports[action[1]-201].swap_time
                blocked = False
                for window in current_node.schedule:
                    if window[0] == action[1]-201:
                        new_window = (time_at_droneport - self.time_thr, time_after_swap + self.time_thr)
                        existing_window = (window[1] - self.time_thr, window[2] + self.time_thr)
                        if max(new_window[0], existing_window[0]) <= min(new_window[1], existing_window[1]):
                            blocked = True
                            break

                if blocked:
                    continue

                # Make sure Droneport has charged battery
                if current_node.n_batt[action[1]-201] <= 0:
                    continue
                # Make sure drone is able to fly
                # and action[2] < end_node.progress[action[0]-1]:
                if action[2] > current_node.progress[action[0]-1]:
                    continue
                # Make sure drone can arrive to Droneport
                if current_node.soc_progress[action[0]-1][action[2]] - action_cost < self.soc_thr:
                    continue

                # Parameters of child node
                # Remove one battery from Droneport
                new_n_batt = current_node.n_batt.copy()
                # TODO: not -1, but transfer value from battery and number of batteries given by threshold on SOC e.g. > 0.9
                new_n_batt[action[1]-201] = new_n_batt[action[1]-201] - 1
                # Recalculate SoC of drone
                new_soc_progress = copy.deepcopy(current_node.soc_progress)
                # TODO: not 1.0 but read value from battery
                new_soc_progress[action[0]-1][action[2]:] = 1.0 - action_cost - \
                    np.cumsum(self.drones[action[0]-1].mission_soc[action[2]:])
                # Update mission progress of drone
                new_progress = current_node.progress.copy()
                new_progress[action[0]-1] = self.drones[action[0]-1].mission_count - 1
                for i, soc in enumerate(new_soc_progress[action[0]-1]):
                    if soc < self.soc_thr:
                        if i == 0:
                            new_progress[action[0]-1] = 0
                        else:
                            new_progress[action[0]-1] = i-1
                        break

                # block Droneport time window in schedule
                time_back = time_after_swap + action_time
                diff_time = time_back - time_at_waypoint
                new_schedule = current_node.schedule.copy()
                new_schedule.append(
                    [action[1]-201, time_at_droneport - self.time_thr, time_after_swap + self.time_thr])
                new_missions_time = copy.deepcopy(current_node.missions_time)
                new_missions_time[action[0]-1][action[2] +
                                            1:] = new_missions_time[action[0]-1][action[2]+1:] + diff_time

                # Create new node
                new_node = self.Node(current_node, new_progress, new_soc_progress,
                                new_n_batt, action, action_cost, action_time, new_missions_time, new_schedule)
                # Append
                children.append(new_node)

            # Loop through children
            for child in children:
                # Child is on the closed list
                if any((child == x).all() for x in self.closed_list):
                    continue

                self.update_node(current_node, child, self.drones, self.open_list)

        warn("Couldn't get a path to destination")
        return None
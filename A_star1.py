'''
***************************************************************************
A* Motion planner for bot from Modern Robotics Course
***************************************************************************
Author: Alexandros Mantzios
Credits: Modern Robotics: Mechanics, Planning, and Control. Code Library
Email: alexandrosmantzios@gmail.com
Date: February 2022
***************************************************************************
Language: Python
***************************************************************************
'''

'''
*** IMPORTS ***
'''

import os.path
import csv
from collections import defaultdict

global PATH

# Set the path for the results folder
PATH = r"C:\Users\alexa\OneDrive\Υπολογιστής\Modern Robotics\Course 4 assignment week 1/results"

# Check if the results folder exists, if not, create it
if not os.path.exists(PATH):
    os.makedirs(PATH)


class OpenNodes:
    """Nodes that are open and need to be revisited"""

    def __init__(self):
        self.dict = {}

    def save(self, value, estimated_total_cost):
        """Save the node and its estimated total cost"""
        self.dict[value] = estimated_total_cost

    def first_node(self):
        """First node based on estimated cost"""
        node = None
        cost = None
        for i, j in self.dict.items():
            if not cost or j < cost:
                node = i
                cost = j
        assert node
        del self.dict[node]
        return node

    def check_empty(self):
        return not bool(self.dict)


class AstarAlgorithm:
    def __init__(self, cost_list, heuristic_cost):
        self.open = OpenNodes()
        self.cost = self.init_cost_list(cost_list)
        # 1st node is the start node
        self.start = heuristic_cost[0][0]
        self.closed = set()
        # last node is the end node
        self.end = heuristic_cost[-1][0]
        self.previous_cost = defaultdict(lambda: 99)
        self.parent_node = {}
        self.heuristic_cost = self.init_heuristic_cost(heuristic_cost)
        # add the start to the open list
        self.open.save(self.start, self.heuristic_cost[self.start])
        # previous cost for start
        self.previous_cost[self.start] = 0

    @staticmethod
    def init_cost_list(cost_list):
        """Initial list"""
        initial_cost = defaultdict(lambda: defaultdict(lambda: None))
        for cost in cost_list:
            start, end, n = cost
            initial_cost[start][end] = n
        result = initial_cost.copy()
        for i in result.keys():
            for j in result[i].keys():
                if i != j:
                    if result[i][j]:
                        initial_cost[j][i] = initial_cost[i][j]
        return initial_cost

    @staticmethod
    def init_heuristic_cost(heuristic_to_go):
        """Initialize list with heuristic cost to go"""
        heuristic_cost_to_go_list = {}
        for i in heuristic_to_go:
            node, cost = i
            heuristic_cost_to_go_list[node] = cost
        return heuristic_cost_to_go_list

    def move_to_next(self, current, next):
        """Look for neighbours"""
        if self.previous_cost[current] is None or self.cost[current][next] is None:
            # Handle cases where the cost is not available
            return
        tentative_past_cost = self.previous_cost[current] + self.cost[current][next]
        if tentative_past_cost < self.previous_cost[next]:
            self.update_path(current, next, tentative_past_cost)

    def next_node(self, node):
        """locate next node in neighborhood"""
        res = set()
        for key, current_node in self.cost.items():
            for next_key, next_node in current_node.items():
                if current_node and next_node and node in (key, next_key):
                    res.add(key)
                    res.add(next_key)
        return res

    def next_node_not_closed(self, node):
        """locate nodes in neighborhood not in closed list"""
        next_nodes = self.next_node(node)
        return [get_node for get_node in next_nodes if get_node not in self.closed]

    def path(self):
        """Find path"""
        print("Starting path finding...")
        while not self.open.check_empty():
            current_node = self.open.first_node()
            if current_node == self.end:
                print("End node reached!")
                return self.save_path(current_node)
            self.closed.add(current_node)
            for next_node in self.next_node_not_closed(current_node):
                print("Processing next node:", next_node)
                self.move_to_next(current_node, next_node)
        print("No valid path found.")
        return None

    def save_path(self, item):
        """Find the optimum path"""
        path = []
        next_node = item
        while next_node != self.start:
            path = [next_node] + path
            next_node = self.parent_node[next_node]
        path = [self.start] + path
        return path

    def update_path(self, current, next, tentative_past_cost):
        self.previous_cost[next] = tentative_past_cost
        self.parent_node[next] = current
        estimated_total_cost = self.previous_cost[next] + self.heuristic_cost[next]
        self.open.save(next, estimated_total_cost)


def read_nodes_csv():
    """Read CSV files "nodes.csv" from the results folder"""
    res = []
    with open(os.path.join(PATH, 'nodes.csv')) as nodes_file:
        nodes = csv.reader(nodes_file)
        for n in nodes:
            try:
                # Attempt to unpack values from the row
                node, _, _, ctg = n
                res.append([int(node), float(ctg)])
            except ValueError:
                # Handle rows with insufficient values
                print("Skipping row with insufficient values:", n)
    return res


def read_edges_csv():
    """Read CSV files "edges.csv" from the results folder"""
    res = []
    with open(os.path.join(PATH, 'edges.csv')) as edges_file:
        edges = csv.reader(edges_file)
        for e in edges:
            try:
                # Attempt to unpack values from the row
                id1, id2, cost = e
                res.append([int(id1), int(id2), float(cost)])
            except ValueError:
                # Handle rows with insufficient values
                print("Skipping row with insufficient values:", e)
    return res


def write_path_csv(result_path):
    # Write to "path.csv" in the results folder
    with open(os.path.join(PATH, 'path.csv'), 'w') as path_f:
        path_csv = csv.writer(path_f)
        path_csv.writerow(result_path)
    print("CSV file 'path.csv' has been created in the 'results' folder.")


if __name__ == "__main__":
    heuristic_cost_final = read_nodes_csv()
    cost = read_edges_csv()
    A_star_algorithm = AstarAlgorithm(cost, heuristic_cost_final)
    best_path = A_star_algorithm.path()
    if best_path is not None:
        write_path_csv(best_path)
    else:
        print("Error")
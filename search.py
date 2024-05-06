# -*- coding: utf-8 -*-
"""
Created on Mon Feb  6 11:45:50 2023

@author: Bijo Sebastian
"""

"""
Implement your search algorithms here
"""

import operator
import math

def heuristic_1(problem, state):
    """
    Euclidean distance
    """
    "*** YOUR CODE HERE ***"
    goal = problem.getGoalState()
    h_cost = math.sqrt((state[0] - goal[0]) ** 2 + (state[1] - goal[1]) ** 2)
    return h_cost
                            
def heuristic_2(problem, state):
    """
    Manhattan distance
    """
    "*** YOUR CODE HERE ***"
    goal = problem.getGoalState()
    h_cost = abs(state[0] - goal[0]) + abs(state[1] - goal[1])
    return h_cost


def weighted_AStarSearch(problem, heuristic_ip):
    """
    Pop the node that having the lowest combined cost plus heuristic
    heuristic_ip can be M, E, or a number 
    if heuristic_ip is M use Manhattan distance as the heuristic function
    if heuristic_ip is E use Euclidean distance as the heuristic function
    if heuristic_ip is a number, use weighted A* Search with Euclidean distance as the heuristic function and the integer being the weight
    """
    "*** YOUR CODE HERE ***"
    import copy
    GPLUSH = 0
    if heuristic_ip == 'E':
        GPLUSH += heuristic_1(problem, problem.getStartState())
    elif heuristic_ip == 'M':
        GPLUSH += heuristic_2(problem, problem.getStartState())
    else:
        GPLUSH += int(heuristic_ip) * heuristic_1(problem, problem.getStartState())
    Fringe = [[problem.getStartState(), [], 0, GPLUSH]]
    Closed_List = []
    while len(Fringe):
        #For popping
        min_index = 0
        for index in range(len(Fringe)):
            if Fringe[index][3] < Fringe[min_index][3]:
                min_index = index
        node = Fringe.pop(min_index)
        
        if problem.isGoalState(node[0]):
            return node[1]
        Closed_List.append(node)
        Successors = problem.getSuccessors(node[0])
        for Successor in Successors:
            Successor_path = copy.deepcopy(node[1])
            Successor_cost = copy.deepcopy(node[2])
            Successor_path.append(Successor[1])
            Successor_cost += Successor[2]
            GPLUSH = Successor_cost
            if heuristic_ip == 'E':
                GPLUSH += heuristic_1(problem, Successor[0])
            elif heuristic_ip == 'M':
                GPLUSH += heuristic_2(problem, Successor[0])
            else:
                GPLUSH += int(heuristic_ip) * heuristic_1(problem, Successor[0])
            Successor_node = [Successor[0], Successor_path, Successor_cost, GPLUSH]
            flag = 0
            for Node in Closed_List:
                if Successor_node[0] == Node[0]:
                    flag = 1
                    break
            if flag:
                continue
            FLAG = 0
            for i in range(len(Fringe)):
                if Successor_node[0] == Fringe[i][0]:
                    if Successor_node[2] < Fringe[i][2]:
                        Fringe.remove(Fringe[i])
                        Fringe.append(Successor_node)
                    FLAG = 1
                    break
            if not FLAG:
                Fringe.append(Successor_node)
    return None

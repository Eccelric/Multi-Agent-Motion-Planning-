import copy
from collections import deque
import maze_maps

import sim_interface
import maze
import random

class DSTARLITE:
    
    class Node:
        def __init__(self, x, y, g = float('inf'), rhs = float('inf')):
            self.x = x
            self.y = y
            self.g = g
            self.rhs = rhs
                
    def __init__(self, problem, states, iden):
        self.problem = problem
        self.open_list = []
        self.km = 0
        self.id = iden
        self.state = states[iden-1]                                             # State contains Start and Goal state
        self.problem.maze_map.map_data[self.state[0][0]][self.state[0][1]] = 3  # Start State
        self.problem.maze_map.map_data[self.state[1][0]][self.state[1][1]] = 8  # Goal State
        self.count = 0
        
    
    def goal_update(self, state):                                               # Update the goal state
        self.problem.maze_map.map_data[self.state[1][0]][self.state[1][1]] = 16
        self.problem.maze_map.map_data[state[0][0]][state[0][1]] = 8
    
    def initialize(self):
        num_rows = len(self.problem.maze_map.map_data)
        num_cols = len(self.problem.maze_map.map_data[0])
        
        self.graph_grid_world = [[self.Node(x, y) for x in range(num_rows)] for y in range(num_cols)]
        self.start_state = self.state[0]
        self.goal_state = self.state[1]
        self.next_state = None                                                   # For deciding to which state to move to next
        self.last_state = None
        self.slast = self.start_state                                                     # Previous state, after making the action
        self.flag = 0
        self.FLAG = 0
        self.to_changed_coords = []                                                 # Stores coordinates that needs to be changed
        self.charge = random.uniform(20, 100)                                       # To store the charge available on the robot
                                                                                    # For each step the robot takes charge is reduced by 0.25 %
        
        
        self.graph_grid_world[self.goal_state[0]][self.goal_state[1]].rhs = 0       # Setting the rhs of goal state to zero
        self.Calculate_key(self.goal_state)                                         # Setting the key of goal state to zero
        self.open_list.append(self.goal_state)                                      # Adding goal state to open list
        
    def h(self, s1, s2):
        return abs(s1[0] - s2[0]) + abs(s1[1] - s2[1])
    
    def is_locally_inconsistent(self, state):                                       # Function to check if the state is locally inconsistent
        x, y = state 
        if self.graph_grid_world[x][y].g == self.graph_grid_world[x][y].rhs:
            return False
        return True
     
    def get_step_cost(self, state):                                             # Gets step cost for travelling to the given state
        if self.problem.maze_map.map_data[state[0]][state[1]] == maze_maps.obstacle_id:
            return float('inf')
        elif self.problem.maze_map.map_data[state[0]][state[1]] == maze_maps.free_space_id2:
            return(maze_maps.free_space_id2_cost)
        elif self.problem.maze_map.map_data[state[0]][state[1]] == maze_maps.free_space_id1 or self.problem.maze_map.map_data[state[0]][state[1]] == maze_maps.goal_id:
            return(maze_maps.free_space_id1_cost) 
        else:
            return(maze_maps.changed_obs_id_cost) 
    
    def update_rhs(self, state):                                                # Function for updating rhs value
        x, y = state
        successors = self.problem.getSuccessors([x, y])                         # [[x, y], required action, step cost]
        self.graph_grid_world[x][y].rhs = float('inf')
        for successor in successors:
            next_state, next_action, _ = successor
            step_cost = self.get_step_cost(next_state)
            g = self.graph_grid_world[next_state[0]][next_state[1]].g
            if g + step_cost < self.graph_grid_world[x][y].rhs:
                self.graph_grid_world[x][y].rhs = g + step_cost
    
    def Calculate_key(self, state):
        x, y = state
        self.graph_grid_world[x][y].key = [min(self.graph_grid_world[x][y].g, self.graph_grid_world[x][y].rhs) + self.h(self.slast, state) + self.km, min(self.graph_grid_world[x][y].g, self.graph_grid_world[x][y].rhs)]
    
    def update_vertex(self, state):
        x, y = state
        print(f"Changed : {x, y}")
        if self.id == 5:
            print(f"Checking : {self.graph_grid_world[x][y].rhs}")
        if x != self.goal_state[0] or y != self.goal_state[1]:
            self.update_rhs(state)
            if self.id == 5:
                print(f"Checking : {self.graph_grid_world[x][y].rhs}")
        if state in self.open_list:
            self.open_list.remove(state)
        
        if self.graph_grid_world[x][y].g != self.graph_grid_world[x][y].rhs:
            self.Calculate_key(state)
            self.open_list.append(state)
        
    def change(self, states):
        for all_states, score in states:
            for state in all_states:            # r_state = (prev_state, current_state)
                self.problem.maze_map.map_data[state[0]][state[1]] = score
    
    def wait_for_changes(self, states):
        old_map = copy.deepcopy(self.problem.maze_map.map_data)
        self.change(states)
        map_changes = [[a - b for a, b in zip(row1, row2)] for row1, row2 in zip(old_map, self.problem.maze_map.map_data)]
        changed_coords = []
        for i in range(len(map_changes)):
            for j in range(len(map_changes[0])):
                if map_changes[i][j] != 0:
                    changed_coords.append([i, j])
        return changed_coords
        
    def action_from_states(self,parent_state,state):
        if(state[0]-parent_state[0]==1):
            return "down"
        elif(state[0]-parent_state[0]==-1):
            return "up"
        elif(state[1]-parent_state[1]==1):
            return "right"
        else:
            return "left"
        
    def extract_path(self):
        state = self.goal_state
        path=deque()
        while(state[0] != self.start_state[0] or state[1] != self.start_state[1]):
            parent_state = self.graph_grid_world[state[0]][state[1]].parent
            action=self.action_from_states(parent_state,state)
            path.appendleft(action)
            state=parent_state
        return path
    
    def action_to_actuation(self, current_state, action):
        current_x, current_y = current_state
        if action == 'up':
            return [current_x - 1.0, current_y]
        elif action == 'down':
            return [current_x + 1.0, current_y]
        elif action == 'right':
            return [current_x, current_y + 1.0]
        elif action == 'left':
            return [current_x, current_y - 1.0]
        else:
            return f"The given action ({action}) is an Invalid action"
        
    def Get_priority_state(self):
        min_state = self.open_list[0]
        min_key = self.graph_grid_world[min_state[0]][min_state[1]].key
        for state in self.open_list:
            key = self.graph_grid_world[state[0]][state[1]].key
            if key[0] < min_key[0] or (key[0] == min_key[0] and key[1] <= min_key[1]):
                min_state = state
                min_key = key
        return min_state
    
    def Compare_keys(self, s1, s2):
        key_1 = self.graph_grid_world[s1[0]][s1[1]].key
        key_2 = self.graph_grid_world[s2[0]][s2[1]].key
        if key_1[0] < key_2[0] or (key_1[0] == key_2[0] and key_1[1] <= key_2[1]):
            return True
        return False
    
    def compute_shortest_path(self):
        self.Calculate_key(self.slast)
        xi = self.Get_priority_state()
        self.count += 1
        while self.Compare_keys(xi, self.slast) or self.is_locally_inconsistent(self.slast):
            kold = self.graph_grid_world[xi[0]][xi[1]].key
            self.open_list.remove(xi)
            self.Calculate_key(xi)
            if kold < self.graph_grid_world[xi[0]][xi[1]].key:
                self.open_list.append(xi)
            elif self.graph_grid_world[xi[0]][xi[1]].g > self.graph_grid_world[xi[0]][xi[1]].rhs:
                self.graph_grid_world[xi[0]][xi[1]].g = self.graph_grid_world[xi[0]][xi[1]].rhs
                successors = self.problem.getSuccessors([xi[0], xi[1]])
                for successor in successors:
                    xj = successor[0]
                    self.update_vertex(xj)
                    
            else:
                self.graph_grid_world[xi[0]][xi[1]].g = float('inf')
                self.update_vertex(xi)
                successors = self.problem.getSuccessors([xi[0], xi[1]])
                for successor in successors:
                    xj = successor[0]
                    self.update_vertex(xj)
            
            if len(self.open_list) != 0:
                xi = self.Get_priority_state()
                self.Calculate_key(self.slast)
                self.count += 1 
        print(f"Number of nodes expanded for robot : {self.id} is : {self.count}")
    
        

    
        
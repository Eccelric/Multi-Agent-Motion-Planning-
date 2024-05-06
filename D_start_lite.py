import copy
from collections import deque
import maze_maps

import sim_interface
import maze

class DSTARLITE:
    
    class Node:
        def __init__(self, x, y, g = float('inf'), rhs = float('inf'), parent = None):
            self.x = x
            self.y = y
            self.g = g
            self.rhs = rhs
            self.key = min(self.g, self.rhs)
            self.parent = parent
            
            
    def __init__(self, problem, states, iden):
        self.problem = problem
        self.open_list = []
        #self.path=[]
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
        self.current_state = None                                               # For deciding to which state to move to next
        self.prev_state = None                                                  # Previous state, after making the action
        self.flag = 0
        self.to_change_coords = []                                              # Stores coordinates that needs to be changed
        
        self.graph_grid_world[self.goal_state[0]][self.goal_state[1]].rhs = 0   # Setting the rhs of goal state to zero
        self.graph_grid_world[self.goal_state[0]][self.goal_state[1]].key = 0   # Setting the key of goal state to zero
        self.open_list.append(self.goal_state)                                  # Adding goal state to open list
    
    def is_locally_inconsistent(self, state):                                   # Function to check if the state is locally inconsistent
        x, y = state
        if self.graph_grid_world[x][y].g == self.graph_grid_world[x][y].rhs:
            return False
        return True
     
    def get_step_cost(self, state):                                             # Gets step cost for travelling to the given state
        if self.problem.maze_map.map_data[state[0]][state[1]] == maze_maps.obstacle_id:
            return float('inf')
        elif self.problem.maze_map.map_data[state[0]][state[1]] == maze_maps.free_space_id2:
            return(maze_maps.free_space_id2_cost)
        elif self.problem.maze_map.map_data[state[0]][state[1]] == maze_maps.free_space_id1 or self.problem.maze_map.map_data[state[0]][state[1]] == maze_maps.start_id:
            return(maze_maps.free_space_id1_cost) 
        else:
            return(maze_maps.changed_obs_id_cost) 
    
    def update_rhs(self, state):                                                # Function for updating rhs value
        x, y = state
        step_cost = self.get_step_cost([x, y])
        successors = self.problem.getSuccessors([x, y])                         # [[x, y], required action, step cost]
        self.graph_grid_world[x][y].rhs = float('inf')
        for successor in successors:
            next_state, next_action, _ = successor
            g = self.graph_grid_world[next_state[0]][next_state[1]].g
            if g + step_cost < self.graph_grid_world[x][y].rhs:
                self.graph_grid_world[x][y].rhs = g + step_cost
                self.graph_grid_world[x][y].parent = next_state
    
    def update_key(self, state):
        x, y = state
        self.graph_grid_world[x][y].key = min(self.graph_grid_world[x][y].g, self.graph_grid_world[x][y].rhs)
    
    def update_vertex(self, state):
        x, y = state
        if x != self.goal_state[0] or y != self.goal_state[1]:
            #print("reached update")
            self.update_rhs(state)
            self.update_key(state)
        
        if self.graph_grid_world[x][y].g != self.graph_grid_world[x][y].rhs:
            #print(2)
            flag = 0
            for s in self.open_list:
                if s[0] == state[0] and s[1] == state[1]:
                    flag = 1
                    break
            if flag == 0:
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
            return [current_x - 0.508, current_y]
        elif action == 'down':
            return [current_x + 0.508, current_y]
        elif action == 'right':
            return [current_x, current_y + 0.508]
        elif action == 'left':
            return [current_x, current_y - 0.508]
        else:
            return f"The given action ({action}) is an Invalid action"
    
    def heuristic(self, obj):
        return abs(obj[0] - self.start_state[0]) + abs(obj[1] - self.start_state[1])
    
    def compute_shortest_path(self):
        xi = min(self.open_list, key = lambda obj : self.graph_grid_world[obj[0]][obj[1]].key + self.heuristic(obj))
        print(xi, self.graph_grid_world[xi[0]][xi[1]].key, self.start_state, self.graph_grid_world[self.start_state[0]][self.start_state[1]].key) 
        self.count += 1
        while self.graph_grid_world[xi[0]][xi[1]].key < self.graph_grid_world[self.start_state[0]][self.start_state[1]].key or self.is_locally_inconsistent(self.start_state):
            if self.graph_grid_world[xi[0]][xi[1]].g > self.graph_grid_world[xi[0]][xi[1]].rhs:
                self.graph_grid_world[xi[0]][xi[1]].g = self.graph_grid_world[xi[0]][xi[1]].rhs
                self.open_list.remove(xi)
            else:
                self.graph_grid_world[xi[0]][xi[1]].g = float('inf')
                self.update_vertex(xi)
                
            successors = self.problem.getSuccessors([xi[0], xi[1]])
            for successor in successors:
                xj = successor[0]
                self.update_vertex(xj)
            
            #print(self.open_list)
            if len(self.open_list) != 0:
                xi = min(self.open_list, key = lambda obj : self.graph_grid_world[obj[0]][obj[1]].key + self.heuristic(obj))
                self.count += 1
            print(xi, self.graph_grid_world[xi[0]][xi[1]].key, self.start_state, self.graph_grid_world[self.start_state[0]][self.start_state[1]].key)
        print(f"Number of nodes expanded is : {self.count}")
    
        

    
        
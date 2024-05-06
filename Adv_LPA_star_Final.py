import copy
from collections import deque
import maze_maps

class LPASTAR:
    
    class Node:
        def __init__(self, x, y, g = float('inf'), rhs = float('inf'), parent = None):
            self.x = x
            self.y = y
            self.g = g
            self.rhs = rhs
            self.key = min(self.g, self.rhs)
            self.parent = parent
            
    def __init__(self, problem):
        self.problem = problem
        self.open_list = []
    
    def initialize(self):
        num_rows = len(self.problem.maze_map.map_data)
        num_cols = len(self.problem.maze_map.map_data[0])
        self.graph_grid_world = [[self.Node(x, y) for x in range(num_rows)] for y in range(num_cols)]
        self.start_state = self.problem.getStartState()
        self.goal_state = self.problem.getGoalState()
        
        self.graph_grid_world[self.start_state[0]][self.start_state[1]].rhs = 0
        self.graph_grid_world[self.start_state[0]][self.start_state[1]].key = 0
        self.open_list.append(self.start_state)
    
    def is_locally_inconsistent(self, state):
        x, y = state
        if self.graph_grid_world[x][y].g == self.graph_grid_world[x][y].rhs:
            return False
        return True
     
    def get_step_cost(self, state):
        if self.problem.maze_map.map_data[state[0]][state[1]] == maze_maps.obstacle_id:
            return float('inf')
        elif self.problem.maze_map.map_data[state[0]][state[1]] == maze_maps.free_space_id2:
            return(maze_maps.free_space_id2_cost)
        elif self.problem.maze_map.map_data[state[0]][state[1]] == maze_maps.free_space_id1 or self.problem.maze_map.map_data[state[0]][state[1]] == maze_maps.goal_id:
            return(maze_maps.free_space_id1_cost) 
        else:
            return(maze_maps.changed_obs_id_cost) 
    
    def update_rhs(self, state):
        x, y = state
        step_cost = self.get_step_cost([x, y])
        successors = self.problem.getSuccessors([x, y])                          #[[x, y], required action, step cost]
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
        if x != self.start_state[0] or y != self.start_state[1]:
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
            """
            self.open_list.append(state)
            """
    def change(self, states):
        for state in states:
            self.problem.maze_map.map_data[state[0]][state[1]] = 16
    
    def wait_for_changes(self):
        old_map = copy.deepcopy(self.problem.maze_map.map_data)
        state = [[0, 4], [0, 5], [0, 6], [0, 7], [0, 8], [0, 9], [0, 10], [1, 4], [1, 5], [1, 6], [1, 7], [1, 8], [1, 9], [1, 10]]
        #state = []
        self.change(state)
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
    
    def lpastar(self):
        self.initialize()
        #xi = min(self.open_list, key = lambda obj : self.graph_grid_world[obj[0]][obj[1]].key)
        while True:
            xi = min(self.open_list, key = lambda obj : self.graph_grid_world[obj[0]][obj[1]].key)
            while self.graph_grid_world[xi[0]][xi[1]].key < self.graph_grid_world[self.goal_state[0]][self.goal_state[1]].key or self.is_locally_inconsistent(self.goal_state):
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
                    xi = min(self.open_list, key = lambda obj : self.graph_grid_world[obj[0]][obj[1]].key)
                print(xi, self.graph_grid_world[xi[0]][xi[1]].key, self.goal_state, self.graph_grid_world[self.goal_state[0]][self.goal_state[1]].key)
                
            path = self.extract_path() 
            print(path)
            
            #return path
            changed_coord = self.wait_for_changes()
            
            if len(changed_coord) == 0:
                return path
            for coord in changed_coord:
                xj = coord
                self.update_vertex(xj)
            #print(self.open_list)
            
        

    
        
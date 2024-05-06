# -*- coding: utf-8 -*-
"""
Created on Mon Feb  6 11:45:50 2023

@author: Bijo Sebastian
"""

import search
import copy
import maze_maps
import matplotlib
#matplotlib.use('Agg')
import matplotlib.pyplot as plt

#Flag to enable plots
#Disbaled in VPL
enable_plots  = True

class Maze:
  """
  This class outlines the structure of the maze problem
  """
  
  maze_map = []# To store map data, start and goal points
  
  # Legal moves
  # [delta_x, delta_y, description]
  four_neighbor_actions = {'up':[-1, 0], 'down':[1, 0], 'left': [0, -1], 'right': [0, 1]}

  if enable_plots:
      #Setup plot
      plt.close('all')
      map_plot_copy = []
      plot_colormap_norm = matplotlib.colors.Normalize(vmin=0.0, vmax=19.0)
      fig,ax = plt.subplots(1)
      plt.axis('equal')
  
  def plot_map(self):
      """
      Plot
      """
      if enable_plots:
          start = self.getStartState()
          goal = self.getGoalState()
          self.map_plot_copy[start[0]][start[1]] = maze_maps.start_id
          self.map_plot_copy[goal[0]][goal[1]] = maze_maps.goal_id
          plt.imshow(self.map_plot_copy, cmap=plt.cm.tab20c, norm=self.plot_colormap_norm)
          plt.show()
      
  # default constructor
  def __init__(self, id):
      """
      Sets the map as defined in file maze_maps
      """
      #Set up the map to be used
      self.maze_map = maze_maps.maps_dictionary[id]
      self.state_expansion_counter = 0
      if enable_plots:
          self.map_plot_copy = copy.deepcopy(self.maze_map.map_data)
          self.plot_map()
      return
     
  def getStartState(self):
     """
     Returns the start state for the search problem 
     """
     start_state = self.maze_map.start
     return start_state
 
  def getGoalState(self):
     """
     Returns the start state for the search problem 
     """
     goal_state =  self.maze_map.goal
     return goal_state
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     if state == self.getGoalState():
         return True
     else:
         return False

  def isObstacle(self, state):
      """
        state: Search state
     
      Returns True if and only if the state is an obstacle
      """
      if self.maze_map.map_data[state[0]][state[1]] == maze_maps.obstacle_id:
          return True
      else:
          return False
  
  def getStateExpansionCount(self):
      """
      Returns number of state expansions
      """
      return self.state_expansion_counter

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     if enable_plots:
         #Update changes on the plot copy
         if self.map_plot_copy[state[0]][state[1]] == maze_maps.fringe_id:
             self.map_plot_copy[state[0]][state[1]] = maze_maps.expanded_id
     
     successors = []
     self.state_expansion_counter = self.state_expansion_counter + 1
     for action in self.four_neighbor_actions:
         
         #Get individual action
         del_x, del_y = self.four_neighbor_actions.get(action)
         
         #Get successor
         new_successor = [state[0] + del_x , state[1] + del_y]
         new_action = action
         
         if new_successor[0] < 0 or new_successor[1] < 0 or new_successor[0] > len(self.maze_map.map_data) - 1 or new_successor[1] > len(self.maze_map.map_data[0]) - 1:
             continue
         
         # Check for obstacle 
         if self.isObstacle(new_successor):
             continue
            
         if enable_plots:
             #Update changes on the plot copy
             if self.map_plot_copy[new_successor[0]][new_successor[1]] == maze_maps.free_space_id1 or self.map_plot_copy[new_successor[0]][new_successor[1]] == maze_maps.free_space_id2:
                 self.map_plot_copy[new_successor[0]][new_successor[1]] = maze_maps.fringe_id
         
         #Check cost
         if self.maze_map.map_data[new_successor[0]][new_successor[1]] == maze_maps.free_space_id2:
             new_cost = maze_maps.free_space_id2_cost
         else:
             new_cost = maze_maps.free_space_id1_cost 
         
         """
         else:
             new_cost = maze_maps.changed_obs_id_cost 
         """
         successors.append([new_successor, new_action, new_cost])
         
     if enable_plots:   
         #Plot the changes
         self.plot_map()
     return successors

if __name__ == '__main__':
    
    current_maze = Maze(1)
    #The three case are listed below, make sure you uncomment each of them and ensure your implimentation works for each 
    #Case a
    #path = search.weighted_AStarSearch(current_maze, 'E')
    #Case b
    #path = search.weighted_AStarSearch(current_maze, 'M')
    #Case c
    path = search.weighted_AStarSearch(current_maze, '10')
    if path:
        #Check path validity
        row,col = current_maze.getStartState() 
        for action in path:
            del_x, del_y = current_maze.four_neighbor_actions.get(action)
            row = row + del_x
            col = col + del_y
            if enable_plots:
                #Update changes on the plot copy
                current_maze.map_plot_copy[row][col] = 10
        if enable_plots:        
            #Plot the solution
            current_maze.plot_map()
        if current_maze.isGoalState([row, col]):
            print("Found a path of ", len(path)," moves by expanding ",current_maze.getStateExpansionCount()," nodes")
        else:
            print('Not a valid path')
        
    else:        
        print("Could not find a path")  

        
    
 
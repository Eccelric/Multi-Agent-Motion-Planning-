# -*- coding: utf-8 -*-
"""
Created on Mon Feb  6 12:16:30 2023

@author: Bijo Sebastian
"""

#Definitions based on color map
start_id = 3
goal_id = 8
obstacle_id = 16
beacon_id = 12
free_space_id1 = 1
free_space_id2 = 18
free_space_id1_cost = 1
free_space_id2_cost = 3
fringe_id = 4
expanded_id = 6
path_id = 10
changed_obs_id = 26
changed_obs_id_cost = float('inf')

class Maps:
    """
    This class outlines the structure of the maps
    """    
    map_data = []
    start = []
    goal = []
    
#Maze maps
map_1 = Maps()
map_1.map_data = [[1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1], 
                  [1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1],
                  [1, 1, 16, 16, 1, 1, 16, 16, 1, 1, 16, 16, 1, 1, 16, 16, 1, 1],
                  [1, 1, 16, 16, 1, 1, 16, 16, 1, 1, 16, 16, 1, 1, 16, 16, 1, 1],
                  [1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1],
                  [1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1],
                  [1, 1, 16, 16, 1, 1, 16, 16, 1, 1, 16, 16, 1, 1, 16, 16, 1, 1],
                  [1, 1, 16, 16, 1, 1, 16, 16, 1, 1, 16, 16, 1, 1, 16, 16, 1, 1],
                  [1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1],
                  [1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1],
                  [1, 1, 16, 16, 1, 1, 16, 16, 1, 1, 16, 16, 1, 1, 16, 16, 1, 1],
                  [1, 1, 16, 16, 1, 1, 16, 16, 1, 1, 16, 16, 1, 1, 16, 16, 1, 1],
                  [1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1],
                  [1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1],
                  [1, 1, 16, 16, 1, 1, 16, 16, 1, 1, 16, 16, 1, 1, 16, 16, 1, 1],
                  [1, 1, 16, 16, 1, 1, 16, 16, 1, 1, 16, 16, 1, 1, 16, 16, 1, 1],
                  [1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1],
                  [1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1,  1,  1, 1, 1]]
map_1.start = [4, 6]
map_1.goal = [9, 8]

map_2 = Maps()
map_2.map_data = [[1, 1,  1,  1], 
                  [1, 1,  1,  1],
                  [1, 16, 16, 8],
                  [3, 1, 1, 1]]

map_2.start = [3, 0]
map_2.goal = [2, 3]

map_3 = Maps()
map_3.map_data = [[1, 1 , 1, 1 , 1, 1 , 1, 1 , 1], 
                  [1, 16, 1, 16, 1, 16, 1, 16, 1],
                  [1, 1 , 1, 1 , 1, 1 , 1, 1 , 1], 
                  [1, 16, 1, 16, 1, 16, 1, 16, 1],
                  [1, 1 , 1, 1 , 1, 1 , 1, 1 , 1], 
                  [1, 16, 1, 16, 1, 16, 1, 16, 1],
                  [1, 1 , 1, 1 , 1, 1 , 1, 1 , 1], 
                  [1, 16, 1, 16, 1, 16, 1, 16, 1],
                  [1, 1 , 1, 1 , 1, 1 , 1, 1 , 1]]
                  
map_3.start = [4, 6]
map_3.goal = [8, 7]

maps_dictionary = {1 : map_1, 2: map_2, 3: map_3}


 

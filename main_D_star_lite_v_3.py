#!/usr/bin/env python

"""
Multi robot simulation setup
@author: Bijo Sebastian 
"""

#Import libraries

#Import files
import maze
import D_start_lite
import sim_interface
import time
import copy
import numpy as np

def not_at_goal_of_robot_all(list_of_robots):
    store_result = []
    for robot in list_of_robots:
        store_result.append(robot.current_state[0] != robot.goal_state[0] or robot.current_state[1] != robot.goal_state[1])
    return np.sum(store_result)

def not_at_goal_of_robot(robot):
    return robot.current_state[0] != robot.goal_state[0] or robot.current_state[1] != robot.goal_state[1]

def get_current_go_to_state(robot):
    successors = robot.problem.getSuccessors(robot.current_state)
    robot.prev_state = robot.current_state
    robot.current_state = min(successors, key = lambda successor : robot.graph_grid_world[successor[0][0]][successor[0][1]].rhs)[0]

def main():
    current_maze = maze.Maze(1)                                                 # Gets the maze details
    
    No_of_Robots = 3
    
    start_states = [[7, 8], [9, 6], [9, 9]]                                     # Start States
    goal_states = [[14, 8], [9, 10], [9, 5]]                                    # Goal States
    
    """
    final_goal_states = []
    for i in range(len(goal_states)):
        final_goal_states.append(min(goal_states, key = lambda x : ((x[0] - start_states[i][0]) ** 2 + (x[1] - start_states[i][1]) ** 2)))
        goal_states.remove(final_goal_states[-1])
    """
    
    # Assigning Start and Goal states
    #states = [[start_states[i], final_goal_states[i]] for i in range(len(start_states))]
    states = [[start_states[i], goal_states[i]] for i in range(len(start_states))]
    print(states)
    
    list_robots = [None for _ in range(No_of_Robots)]                           # Stores robot D_star_lite class for all robots
    
    for i in range(len(list_robots)):
        list_robots[i] = D_start_lite.DSTARLITE(current_maze, states, iden = i + 1)
        
    if sim_interface.sim_init():
        for i in range(len(list_robots)):
            list_robots[i].robot = sim_interface.Pioneer(list_robots[i].id)
            list_robots[i].robot.localize_robot()
            list_robots[i].old_goal = [list_robots[i].current_state[0], list_robots[i].current_state[1]]
    else:
        print ('Failed connecting to remote API server')
        
    for i in range(len(list_robots)):
        list_robots[i].initialize()
        list_robots[i].compute_shortest_path()
        list_robots[i].current_state = list_robots[i].start_state
        
    while not_at_goal_of_robot_all(list_robots):
        
        for i in range(len(list_robots)):
            robot = list_robots[i]
            if robot.graph_grid_world[robot.start_state[0]][robot.start_state[1]].g == float('inf'):
                print(f"No known path for robot {i + 1}")
    
            if not_at_goal_of_robot(robot):
                get_current_go_to_state(robot)
                print(f"robot {i + 1} : {robot.current_state}")
            else:
                robot.flag = 1
            print(f"robot {i + 1} : {robot.current_state}")
        
        
    
#run
if __name__ == '__main__':

    main()                    
    print ('Program ended')
            

 
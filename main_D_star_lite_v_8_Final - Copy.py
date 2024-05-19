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

def not_all_at_goal(list_robots):
    bool_1 = not list_robots[0].robot.robot_at_goal()
    for i in range(1, len(list_robots)):
        bool_1 = bool_1 or not list_robots[i].robot.robot_at_goal()
    return bool_1

def move_to(list_robots):
    if sim_interface.start_simulation():
        for robot in list_robots:
            robot.robot.localize_robot()
            if not robot.flag:
                action = robot.action_from_states(robot.last_state, robot.next_state)
                robot.robot.goal_state = robot.action_to_actuation(robot.old_goal, action)
                robot.old_goal = robot.robot.goal_state
        start_time = time.time()
        curr_time = time.time()
        
        for robot in list_robots:
            #if not robot.flag:
            robot.charge -= 0.5
            #else:
            #    if robot.charge < 100:
            #        robot.charge += 0.5
        
        while not_all_at_goal(list_robots):
            if curr_time - start_time > 20.0:
                break
            for robot in list_robots:
                if not robot.flag:
                    robot.problem.map_plot_copy[robot.next_state[0]][robot.next_state[1]] = 10
                    robot.robot.run_controller()
            curr_time = time.time()
        for robot in list_robots:
            robot.robot.setvel_pioneer(0.0, 0.0)
    else:
        print('Failed to start Simulation')

def collision_checker(list_robots):
    
    for robot in list_robots:
        if len(robot.to_changed_coords) != 0:
            for coord in robot.to_changed_coords:
                FLAG = 0
                for frobot in list_robots:
                    if frobot.flag == 1:
                        if frobot.last_state[0] == coord[0][0] and frobot.last_state[1] == coord[0][1]:
                            FLAG = 1
                if FLAG == 0:
                    coord[1] = 1                                                     # Convert from obstacle index to free index
                    robot.FLAG += 1
            changed_coords_robot = robot.wait_for_changes(robot.to_changed_coords)
            robot.update_vertex(robot.last_state)
            robot.compute_shortest_path()
            robot.next_state = robot.last_state
            get_current_go_to_state(robot)     
            robot.to_changed_coords = []
        
    list_robots.sort(key = lambda x : x.charge)
    
    for robot in list_robots:
        if robot.flag == 1:
            list_robots.remove(robot)
            list_robots.insert(0, robot)
    for i in range(len(list_robots)):
        robot = list_robots[i]
        for j in range(len(list_robots)):
            sub_robot = list_robots[j]
            if j <= i or sub_robot.flag == 1:
                continue
            
            if robot.flag == 1 and robot.last_state == sub_robot.next_state:
                print(f"Executed,{robot.id, sub_robot.id}")
                sub_robot.to_changed_coords.append([[robot.last_state], 16])
                sub_robot.FLAG += 1
                
            if robot.next_state == sub_robot.next_state:
                print(f"Executed,{robot.id, sub_robot.id}")
                sub_robot.to_changed_coords.append([[robot.next_state], 16])
                sub_robot.FLAG += 1
            if robot.last_state == sub_robot.next_state and robot.next_state == sub_robot.last_state:
                sub_robot.to_changed_coords.append([[robot.last_state], 16])
                sub_robot.FLAG += 1
            if sub_robot.FLAG == 1:
                sub_robot.km += sub_robot.h(sub_robot.slast, sub_robot.last_state)
                sub_robot.slast = sub_robot.last_state                
            changed_coords_sub_robot = sub_robot.wait_for_changes(sub_robot.to_changed_coords)
            if sub_robot.id == 5:
                print("Collision")
                print(f"g : {sub_robot.graph_grid_world[sub_robot.last_state[0]][sub_robot.last_state[1]].g}")
                print(f"rhs: {sub_robot.graph_grid_world[sub_robot.last_state[0]][sub_robot.last_state[1]].rhs}")
            sub_robot.update_vertex(sub_robot.last_state)
            if sub_robot.id == 5:
                print("Collision")
                print(f"g : {sub_robot.graph_grid_world[sub_robot.last_state[0]][sub_robot.last_state[1]].g}")
                print(f"rhs: {sub_robot.graph_grid_world[sub_robot.last_state[0]][sub_robot.last_state[1]].rhs}")
                
            sub_robot.compute_shortest_path()
            sub_robot.next_state = sub_robot.last_state
            get_current_go_to_state(sub_robot)

def not_at_goal_of_robot_all(list_of_robots):
    store_result = []
    for robot in list_of_robots:
        store_result.append(robot.next_state[0] != robot.goal_state[0] or robot.next_state[1] != robot.goal_state[1])
    return np.sum(store_result)

def not_at_goal_of_robot(robot):
    return robot.next_state[0] != robot.goal_state[0] or robot.next_state[1] != robot.goal_state[1]

def get_current_go_to_state(robot):
    successors = robot.problem.getSuccessors(robot.next_state)
    robot.last_state = robot.next_state
    robot.next_state = min(successors, key = lambda successor : robot.get_step_cost(successor[0]) + robot.graph_grid_world[successor[0][0]][successor[0][1]].g)[0]

def main():
    #current_maze = maze.Maze(1)                                                 # Gets the maze details
    
    No_of_Robots = 5
    
    list_current_maze = [None for _ in range(No_of_Robots)]
    
        
    
    start_states = [[7, 8], [9, 6], [9, 9], [8, 11], [11, 8]]                                     # Start States
    goal_states = [[1, 16], [1, 13], [4, 13], [4, 16], [6, 16]]                                   # Goal States                                 # Goal States
    #start_states = [[2, 7], [4, 7], [4, 6]]                                     # Start States
    #goal_states = [[3, 8], [4, 8], [5, 8]]                                   # Goal States 
    
    final_goal_states = []
    for i in range(len(goal_states)):
        final_goal_states.append(min(goal_states, key = lambda x : ((x[0] - start_states[i][0]) ** 2 + (x[1] - start_states[i][1]) ** 2)))
        goal_states.remove(final_goal_states[-1])
    
    
    # Assigning Start and Goal states
    states = [[start_states[i], final_goal_states[i]] for i in range(len(start_states))]
    #states = [[start_states[i], goal_states[i]] for i in range(len(start_states))]
    for i in range(No_of_Robots):
        list_current_maze[i] = maze.Maze(1, states[i], i + 1) 
        
    list_robots = [None for _ in range(No_of_Robots)]                           # Stores robot D_star_lite class for all robots
     
    for i in range(len(list_robots)):
        list_robots[i] = D_start_lite.DSTARLITE(list_current_maze[i], states, iden = i + 1)
    if sim_interface.sim_init():
        for i in range(len(list_robots)):
            list_robots[i].robot = sim_interface.Pioneer(list_robots[i].id)
            list_robots[i].robot.localize_robot()
            list_robots[i].old_goal = [list_robots[i].robot.current_state[0], list_robots[i].robot.current_state[1]]
    else:
        print ('Failed connecting to remote API server')
        
    for i in range(len(list_robots)):
        list_robots[i].initialize()
        list_robots[i].next_state = list_robots[i].start_state
        list_robots[i].slast = list_robots[i].next_state
        list_robots[i].last_state = list_robots[i].next_state
        list_robots[i].compute_shortest_path()

    for i in range(len(list_robots)):
        while not_at_goal_of_robot(list_robots[i]):
            if list_robots[i].graph_grid_world[list_robots[i].last_state[0]][list_robots[i].last_state[1]].g == float('inf'):
                print(f"No known path for robot {i + 1}")
            if not_at_goal_of_robot(list_robots[i]):
                get_current_go_to_state(list_robots[i])
            else:
                list_robots[i].flag = 1
                list_robots[i].last_state = list_robots[i].next_state
                
                for sub_robot in list_robots:
                    sub_robot.problem.map_plot_copy[list_robots[i].next_state[0]][list_robots[i].next_state[1]] = 16
        
    for robot in list_robots:
        if robot.id == 5:
            print(f"g : {robot.graph_grid_world[robot.last_state[0]][robot.last_state[1]].g}")
            print(f"rhs: {robot.graph_grid_world[robot.last_state[0]][robot.last_state[1]].rhs}")
    
    
    collision_checker(list_robots)
    
    
    for robot in list_robots:
        if robot.id == 5:
            print(f"g : {robot.graph_grid_world[robot.last_state[0]][robot.last_state[1]].g}")
            print(f"rhs: {robot.graph_grid_world[robot.last_state[0]][robot.last_state[1]].rhs}")
    
    for i in range(len(list_robots)):
        robot = list_robots[i]

    for robot in list_robots:
        if robot.id == 5:
            print(robot.last_state, robot.next_state)

    move_to(list_robots)
        
    time.sleep(5.0)
    sim_interface.sim_shutdown()
    time.sleep(2.0)
    
#run
if __name__ == '__main__':

    main()                    
    print ('Program ended')
            

 
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
                action = robot.action_from_states(robot.prev_state, robot.current_state)
                #robot.path.append(action)
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
                    robot.problem.map_plot_copy[robot.current_state[0]][robot.current_state[1]] = 10
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
                coord[1] = 1                                                     # Convert from obstacle index to free index
            changed_coords_robot = robot.wait_for_changes(robot.to_changed_coords)
            for coord in changed_coords_robot:
                xj = coord
                robot.update_vertex(xj)
            robot.compute_shortest_path()
            robot.current_state = robot.prev_state
            get_current_go_to_state(robot)     
        
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
            if robot.current_state == sub_robot.current_state:
                print(f"Executed,{robot.id, sub_robot.id}")
                sub_robot.to_changed_coords.append([[robot.current_state], 16])
            if robot.prev_state == sub_robot.current_state and robot.current_state == sub_robot.prev_state:
                sub_robot.to_changed_coords.append([[robot.prev_state], 16])
            
            changed_coords_sub_robot = sub_robot.wait_for_changes(sub_robot.to_changed_coords)
            for coord in changed_coords_sub_robot:
                xj = coord
                sub_robot.update_vertex(xj)
            sub_robot.compute_shortest_path()
            sub_robot.current_state = sub_robot.prev_state
            get_current_go_to_state(sub_robot)

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
    #current_maze = maze.Maze(1)                                                 # Gets the maze details
    
    No_of_Robots = 3
    
    list_current_maze = [None for _ in range(No_of_Robots)]
    
        
    
    start_states = [[3, 4], [4, 4], [4, 3]]                                    # Start States
    goal_states = [[4, 4], [2, 4], [4, 5]]                                     # Goal States                                 # Goal States
    
    """
    final_goal_states = []
    for i in range(len(goal_states)):
        final_goal_states.append(min(goal_states, key = lambda x : ((x[0] - start_states[i][0]) ** 2 + (x[1] - start_states[i][1]) ** 2)))
        goal_states.remove(final_goal_states[-1])
    """
    
    # Assigning Start and Goal states
    #states = [[start_states[i], final_goal_states[i]] for i in range(len(start_states))]
    states = [[start_states[i], goal_states[i]] for i in range(len(start_states))]
    for i in range(No_of_Robots):
        list_current_maze[i] = maze.Maze(3, states[i], i + 1) 
        
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
        
    list_robots.sort(key = lambda x : x.charge)
    
    for i in range(len(list_robots)):
        list_robots[i].compute_shortest_path()
        list_robots[i].current_state = list_robots[i].start_state

    for robot in list_robots:
        print(f"{robot.id} : {states[robot.id - 1]}")

    #for robot in list_robots:
    #    print(f"The robot {robot.id} available charge is : {robot.charge}")
        
    while not_at_goal_of_robot_all(list_robots):
        for i in range(len(list_robots)):
            robot = list_robots[i]
            if robot.graph_grid_world[robot.start_state[0]][robot.start_state[1]].g == float('inf'):
                print(f"No known path for robot {i + 1}")
    
            if not_at_goal_of_robot(robot):
                get_current_go_to_state(robot)
            else:
                robot.flag = 1
            
        collision_checker(list_robots)
        
        for i in range(len(list_robots)):
            robot = list_robots[i]
          
        move_to(list_robots)
        

        #for robot in list_robots:
        #    print(f"The robot {robot.id} available charge is : {robot.charge}")
        
        flags = list_robots[0].flag
        for i in range(1, len(list_robots)):
            flags = flags and list_robots[i].flag
        if flags:
            break
        
    time.sleep(5.0)
    sim_interface.sim_shutdown()
    time.sleep(2.0)
    
#run
if __name__ == '__main__':

    main()                    
    print ('Program ended')
            

 
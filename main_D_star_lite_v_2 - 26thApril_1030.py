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

def move_to(robot_1s, robot_2s, robot_3s):
    robot_1, flag_1 = robot_1s
    robot_2, flag_2 = robot_2s
    robot_3, flag_3 = robot_3s
    
    if (sim_interface.start_simulation()):
        
        robot_1.robot.localize_robot()
        robot_2.robot.localize_robot()
        robot_3.robot.localize_robot()
        
        if not flag_1:
            action_1 = robot_1.action_from_states(robot_1.prev_state, robot_1.current_state)
            robot_1.path.append(action_1)
            robot_1.robot.goal_state = robot_1.action_to_actuation(robot_1.old_goal, action_1)
            robot_1.old_goal = robot_1.robot.goal_state
            
        if not flag_2:
            action_2 = robot_2.action_from_states(robot_2.prev_state, robot_2.current_state)
            robot_2.path.append(action_2)
            robot_2.robot.goal_state = robot_2.action_to_actuation(robot_2.old_goal, action_2)
            robot_2.old_goal = robot_2.robot.goal_state
            
        if not flag_3:
            action_3 = robot_3.action_from_states(robot_3.prev_state, robot_3.current_state)
            robot_3.path.append(action_3)
            robot_3.robot.goal_state = robot_3.action_to_actuation(robot_3.old_goal, action_3)
            robot_3.old_goal = robot_3.robot.goal_state
        start_time = time.time()
        curr_time = time.time()
        while not robot_1.robot.robot_at_goal() or not robot_2.robot.robot_at_goal() or not robot_3.robot.robot_at_goal():
            if curr_time - start_time > 20.0:
                break
            #Run the control loops for three robots
            if not flag_1:
                robot_1.robot.run_controller()
            if not flag_2:
                robot_2.robot.run_controller()
            if not flag_3:
                robot_3.robot.run_controller()
            curr_time = time.time()
            print(curr_time - start_time)
            """
            print(f"Grid world : {robot_1.current_state}, {robot_1.goal_state}") 
            print(f"Grid world : {robot_2.current_state}, {robot_2.goal_state}") 
            print(f"Grid world : {robot_3.current_state}, {robot_3.goal_state}") 
            print(robot_1.robot.current_state, robot_1.robot.goal_state, robot_1.old_goal) 
            print(robot_2.robot.current_state, robot_2.robot.goal_state, robot_2.old_goal) 
            print(robot_3.robot.current_state, robot_3.robot.goal_state, robot_3.old_goal) 
            """
        robot_1.robot.setvel_pioneer(0.0, 0.0)
        robot_2.robot.setvel_pioneer(0.0, 0.0)
        robot_3.robot.setvel_pioneer(0.0, 0.0)
        
    else:
        print ('Failed to start simulation')
    
def not_at_goal_of_robot(robot):
    return robot.current_state[0] != robot.goal_state[0] or robot.current_state[1] != robot.goal_state[1]

def get_current_go_to_state(robot):
    successors = robot.problem.getSuccessors(robot.current_state)
    robot.prev_state = robot.current_state
    robot.current_state = min(successors, key = lambda successor : robot.graph_grid_world[successor[0][0]][successor[0][1]].rhs)[0]

def collision_checker(robot_1l, robot_2l, robot_3l, goal_states):
    robot_1, id_1, flag_1 = robot_1l
    robot_2, id_2, flag_2 = robot_2l
    robot_3, id_3, flag_3 = robot_3l
    print("collision checker")
    
    to_change_coords_2 = [[[robot_1.prev_state, robot_3.prev_state], 1]]
    to_change_coords_3 = [[[robot_2.prev_state, robot_1.prev_state], 1]]
    
    changed_coords_2 = []
    changed_coords_3 = []
    
    if flag_2 == 0:
        if robot_2.current_state == robot_1.current_state:     
            if robot_1.goal_state == robot_2.current_state:
                g_states = copy.deepcopy(goal_states)
                g_states.remove(robot_1.goal_state)
                robot_2.goal_state = min(g_states, key = lambda x : ((x[0] - robot_2.prev_state[0]) ** 2 + (x[1] - robot_2.prev_state[1]) ** 2))     
            to_change_coords_2.append([[robot_1.current_state], 16])
        changed_coords_2 = robot_2.wait_for_changes(to_change_coords_2)    
        for coord in changed_coords_2:
            xj = coord
            robot_2.update_vertex(xj)
        robot_2.compute_shortest_path()
        robot_2.current_state = robot_2.prev_state
        get_current_go_to_state(robot_2)
        
    if flag_3 == 0:
        if robot_3.current_state == robot_1.current_state:      
            if robot_1.goal_state == robot_3.current_state:
                g_states = copy.deepcopy(goal_states)
                g_states.remove(robot_1.goal_state)
                robot_3.goal_state = min(g_states, key = lambda x : ((x[0] - robot_3.prev_state[0]) ** 2 + (x[1] - robot_3.prev_state[1]) ** 2))
            to_change_coords_3.append([[robot_1.current_state], 16])
        
            
        if robot_3.current_state == robot_2.current_state:  
            if robot_2.goal_state == robot_3.current_state:
                g_states = copy.deepcopy(goal_states)
                g_states.remove(robot_2.goal_state)
                robot_3.goal_state = min(g_states, key = lambda x : ((x[0] - robot_3.prev_state[0]) ** 2 + (x[1] - robot_3.prev_state[1]) ** 2))  
            to_change_coords_3.append([[robot_2.current_state], 16])
        changed_coords_3 = robot_3.wait_for_changes(to_change_coords_3)
        for coord in changed_coords_3:
            xj = coord
            robot_3.update_vertex(xj)
        robot_3.compute_shortest_path()
        robot_3.current_state = robot_3.prev_state
        get_current_go_to_state(robot_3)
        
def main():
    current_maze = maze.Maze(1)
    
    start_states = [[7, 8], [9, 6], [9, 10]]                                    # Defining Start states
    goal_states = [[14, 8], [12, 10], [7, 4]]                                      # Defining Goal states
    
    final_goal_states = []
    for i in range(len(goal_states)):
        final_goal_states.append(min(goal_states, key = lambda x : ((x[0] - start_states[i][0]) ** 2 + (x[1] - start_states[i][1]) ** 2)))
        goal_states.remove(final_goal_states[-1])
    
    #Assigning Start and Goal states
    states = [[start_states[i], final_goal_states[i]] for i in range(len(start_states))]
    #states = [[start_states[i], goal_states[i]] for i in range(len(start_states))]
    print(states)
    
    list_robots = [None for _ in range(3)]
    for i in range(3):
        start, goal = states[i]
        dist = (start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2
        list_robots[i] = [i + 1, dist]
    
    first = min(list_robots, key = lambda x : x[1])                              # Robot with first preference
    list_robots.remove(first)    
    second = min(list_robots, key = lambda x : x[1])                             # Robot with second preference
    list_robots.remove(second)
    third = list_robots[0]                                                       # Robot with third preference
    
    robot_1 = D_start_lite.DSTARLITE(current_maze, states, iden = 1)             # Defining D star lite class for robot 1
    robot_2 = D_start_lite.DSTARLITE(current_maze, states, iden = 2)             # Defining D star lite class for robot 2
    robot_3 = D_start_lite.DSTARLITE(current_maze, states, iden = 3)             # Defining D star lite class for robot 3
    
    if (sim_interface.sim_init()):                                               # Starts the simulation

        # Create three robot and setup interface for all three 
        robot_1.robot = sim_interface.Pioneer(robot_1.id)
        robot_2.robot = sim_interface.Pioneer(robot_2.id)
        robot_3.robot = sim_interface.Pioneer(robot_3.id)
        
        # Localization of all three robots
        robot_1.robot.localize_robot()
        robot_2.robot.localize_robot()
        robot_3.robot.localize_robot()
        
        robot_1.old_goal = [robot_1.robot.current_state[0], robot_1.robot.current_state[1]]
        robot_2.old_goal = [robot_2.robot.current_state[0], robot_2.robot.current_state[1]]
        robot_3.old_goal = [robot_3.robot.current_state[0], robot_3.robot.current_state[1]]
    
    else:
        print ('Failed connecting to remote API server')
        
    # Initialize all Robots
    robot_1.initialize()
    robot_2.initialize()
    robot_3.initialize()
    
    # Compute shortest path for all robots
    robot_1.compute_shortest_path()
    robot_2.compute_shortest_path()
    robot_3.compute_shortest_path()
    
    robot_1.current_state = robot_1.start_state
    robot_2.current_state = robot_2.start_state
    robot_3.current_state = robot_3.start_state
    
    flag_1 = 0
    flag_2 = 0
    flag_3 = 0
    
    while not_at_goal_of_robot(robot_1) or not_at_goal_of_robot(robot_2) or not_at_goal_of_robot(robot_3):
        
        if robot_1.graph_grid_world[robot_1.start_state[0]][robot_1.start_state[1]].g == float('inf'):
            print("No known path for robot 1")
        if robot_2.graph_grid_world[robot_2.start_state[0]][robot_2.start_state[1]].g == float('inf'):
            print("No known path for robot 2")
        if robot_3.graph_grid_world[robot_3.start_state[0]][robot_3.start_state[1]].g == float('inf'):
            print("No known path for robot 3")
        
        if not_at_goal_of_robot(robot_1):
            get_current_go_to_state(robot_1)
            print(f"robot_1 : {robot_1.current_state}")
        else:
            flag_1 = 1
        if not_at_goal_of_robot(robot_2):
            get_current_go_to_state(robot_2)
            print(f"robot_2 : {robot_2.current_state}")
        else:
            flag_2 = 1
        if not_at_goal_of_robot(robot_3):
            get_current_go_to_state(robot_3)
            print(f"robot_3 : {robot_3.current_state}")
        else:
            flag_3 = 1

        
        list_robots = [None for _ in range(3)]
        robots = [robot_1.prev_state, robot_2.prev_state, robot_3.prev_state]
        for i in range(3):
            _, goal = states[i]
            start = robots[i]
            dist = (start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2
            list_robots[i] = [i + 1, dist]
    
        first = min(list_robots, key = lambda x : x[1])                              # Robot with first preference
        list_robots.remove(first)    
        second = min(list_robots, key = lambda x : x[1])                             # Robot with second preference
        list_robots.remove(second)
        third = list_robots[0]
        print(first, second, third)

        """
        if first[0] == 1 and second[0] == 2:
            collision_checker([robot_1, 0], [robot_2, 1], [robot_3, 2], goal_states)
        elif first[0] == 2 and second[0] == 1:
            collision_checker([robot_2, 1], [robot_1, 0], [robot_3, 2], goal_states)
        elif second[0] == 1 and third[0] == 2:
            collision_checker([robot_2, 1], [robot_3, 2], [robot_1, 0], goal_states)
        elif third[0] == 1 and second[0] == 2:
            collision_checker([robot_3, 2], [robot_2, 1], [robot_1, 0], goal_states)
        elif third[0] == 1 and first[0] == 2:
            collision_checker([robot_3, 2], [robot_1, 0], [robot_2, 1], goal_states)
        else:
            collision_checker([robot_1, 0], [robot_3, 2], [robot_2, 1], goal_states)
        """
        collision_checker([robot_1, 0, flag_1], [robot_2, 1, flag_2], [robot_3, 2, flag_3], goal_states)
        move_to([robot_1, flag_1], [robot_2, flag_2], [robot_3, flag_3])
        
        print(f"flag_1 : {flag_1}, flag_2 : {flag_2}, flag_3 : {flag_3}")
        if flag_1 and flag_2 and flag_3:
            break
    print(f"flag_1 : {flag_1}, flag_2 : {flag_2}, flag_3 : {flag_3}")    
    time.sleep(5.0)
    sim_interface.sim_shutdown()
    time.sleep(2.0)
    
    
#run
if __name__ == '__main__':

    main()                    
    print ('Program ended')
            

 
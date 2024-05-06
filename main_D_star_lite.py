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

def move_to(robot_1, robot_2, robot_3):
    if (sim_interface.start_simulation()):
        robot_1.robot1.localize_robot()
        robot_2.robot1.localize_robot()
        robot_3.robot1.localize_robot()
        action_1 = robot_1.action_from_states(robot_1.prev_state, robot_1.current_state)
        robot_1.path.append(action_1)
        action_2 = robot_2.action_from_states(robot_2.prev_state, robot_2.current_state)
        robot_2.path.append(action_2)
        action_3 = robot_3.action_from_states(robot_3.prev_state, robot_3.current_state)
        robot_3.path.append(action_3)
        robot_1.robot1.goal_state = robot_1.action_to_actuation(robot_1.old_goal, action_1)
        robot_1.old_goal = robot_1.robot1.goal_state
        robot_2.robot1.goal_state = robot_2.action_to_actuation(robot_2.old_goal, action_2)
        robot_2.old_goal = robot_2.robot1.goal_state
        robot_3.robot1.goal_state = robot_3.action_to_actuation(robot_3.old_goal, action_3)
        robot_3.old_goal = robot_3.robot1.goal_state
        while not robot_1.robot1.robot_at_goal() or not robot_2.robot1.robot_at_goal() or not robot_3.robot1.robot_at_goal():
        #Run the control loops for three robots
            robot_1.robot1.run_controller()
            robot_2.robot1.run_controller()
            robot_3.robot1.run_controller()
            #print("Robot1")
            print(robot_1.robot1.current_state, robot_1.robot1.goal_state, robot_1.old_goal) 
            #print("Robot2")
            print(robot_2.robot1.current_state, robot_2.robot1.goal_state, robot_2.old_goal) 
            #print("Robot3")
            print(robot_3.robot1.current_state, robot_3.robot1.goal_state, robot_3.old_goal) 
        
        robot_1.robot1.setvel_pioneer(0.0, 0.0)
        robot_2.robot1.setvel_pioneer(0.0, 0.0)
        robot_3.robot1.setvel_pioneer(0.0, 0.0)
        
    else:
        print ('Failed to start simulation')
    
def not_at_goal_of_robot(robot):
    return robot.current_state[0] != robot.goal_state[0] or robot.current_state[1] != robot.goal_state[1]

def get_current_go_to_state(robot):
    successors = robot.problem.getSuccessors(robot.current_state)
    robot.prev_state = robot.current_state
    robot.current_state = min(successors, key = lambda successor : robot.graph_grid_world[successor[0][0]][successor[0][1]].rhs)[0]

def collision_checker(robot_1, robot_2, robot_3, goal_states):
    if robot_2.current_state == robot_1.current_state:
        """
        if robot_1.goal_state == robot_2.current_state:
            g_states = copy.deepcopy(goal_states)
            g_states.remove(robot_1.goal_state)
            robot_2.goal_state = min(g_states, key = lambda x : ((x[0] - robot_2.current_state[0]) ** 2 + (x[1] - robot_2.current_state[1]) ** 2))
            """
        changed_coords_2 = robot_2.wait_for_changes([robot_1.current_state])
        for coord in changed_coords_2:
            xj = coord
            robot_2.update_vertex(xj)
        robot_2.compute_shortest_path()
        get_current_go_to_state(robot_2)
        
    if robot_3.current_state == robot_1.current_state:
        """
        if robot_1.goal_state == robot_3.current_state:
            g_states = copy.deepcopy(goal_states)
            g_states.remove(robot_1.goal_state)
            robot_3.goal_state = min(g_states, key = lambda x : ((x[0] - robot_3.current_state[0]) ** 2 + (x[1] - robot_3.current_state[1]) ** 2))
          """
        
        changed_coords_3 = robot_3.wait_for_changes([robot_1.current_state])
        for coord in changed_coords_3:
            xj = coord
            robot_3.update_vertex(xj)
        robot_3.compute_shortest_path()
        get_current_go_to_state(robot_3)
        
    if robot_3.current_state == robot_2.current_state:
        """
        if robot_2.goal_state == robot_3.current_state:
            g_states = copy.deepcopy(goal_states)
            g_states.remove(robot_2.goal_state)
            robot_3.goal_state = min(g_states, key = lambda x : ((x[0] - robot_3.current_state[0]) ** 2 + (x[1] - robot_3.current_state[1]) ** 2))
         
        """
        changed_coords_3 = robot_3.wait_for_changes([robot_2.current_state])
        for coord in changed_coords_3:
            xj = coord
            robot_3.update_vertex(xj)
        robot_3.compute_shortest_path()
        get_current_go_to_state(robot_3)

def main():
    current_maze = maze.Maze(1)
    #path = search.weighted_AStarSearch(current_maze, 'M')
    start_states = [[4, 6], [0, 0], [17, 17]]
    goal_states = [[9, 8], [0, 5], [4, 16]]
    states = [[start_states[0], min(goal_states, key = lambda x : ((x[0] - start_states[0][0]) ** 2 + (x[1] - start_states[0][1]) ** 2))], 
              [start_states[1], min(goal_states, key = lambda x : ((x[0] - start_states[1][0]) ** 2 + (x[1] - start_states[1][1]) ** 2))], 
              [start_states[2], min(goal_states, key = lambda x : ((x[0] - start_states[2][0]) ** 2 + (x[1] - start_states[2][1]) ** 2))]]
    robot_1 = D_start_lite.DSTARLITE(current_maze, states, iden = 1)
    robot_2 = D_start_lite.DSTARLITE(current_maze, states, iden = 2)
    robot_3 = D_start_lite.DSTARLITE(current_maze, states, iden = 3)  
    
    if (sim_interface.sim_init()):
        #sim_interface.sim_shutdown()

        #Create three robot and setup interface for all three 
        robot_1.robot1 = sim_interface.Pioneer(robot_1.id)
        robot_1.robot1.localize_robot()
        robot_1.old_goal = [robot_1.robot1.current_state[0], robot_1.robot1.current_state[1]]
    
        robot_2.robot1 = sim_interface.Pioneer(robot_2.id)
        robot_2.robot1.localize_robot()
        robot_2.old_goal = [robot_2.robot1.current_state[0], robot_2.robot1.current_state[1]]
    
        robot_3.robot1 = sim_interface.Pioneer(robot_3.id)
        robot_3.robot1.localize_robot()
        robot_3.old_goal = [robot_3.robot1.current_state[0], robot_3.robot1.current_state[1]]
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
    
    while not_at_goal_of_robot(robot_1) or not_at_goal_of_robot(robot_2) or not_at_goal_of_robot(robot_3):
        
        if robot_1.graph_grid_world[robot_1.start_state[0]][robot_1.start_state[1]].g == float('inf'):
            print("No known path for robot 1")
        if robot_2.graph_grid_world[robot_2.start_state[0]][robot_2.start_state[1]].g == float('inf'):
            print("No known path for robot 2")
        if robot_3.graph_grid_world[robot_3.start_state[0]][robot_3.start_state[1]].g == float('inf'):
            print("No known path for robot 3")
        
        if not_at_goal_of_robot(robot_1):
            get_current_go_to_state(robot_1)
        if not_at_goal_of_robot(robot_2):
            get_current_go_to_state(robot_2)
        if not_at_goal_of_robot(robot_3):
            get_current_go_to_state(robot_3)
        
        collision_checker(robot_1, robot_2, robot_3, goal_states)
        
        move_to(robot_1, robot_2, robot_3)
        
    time.sleep(5.0)
    sim_interface.sim_shutdown()
    time.sleep(2.0)
    
    
#run
if __name__ == '__main__':

    main()                    
    print ('Program ended')
            

 
"""
initialize start to slast for all robots
initialize all robots
Compute shortest path for all robots
while all robots have not reached goal location.
for all robots  if g(start) = infinity then no path exists
get next state for each robot
Run collision checker for all robots based on priority given based on battery
make the changes in the graph
increase km and equate slast to the new current state
update vertex
repeat from step3
"""

#Import files
import maze
import D_start_lite
import sim_interface
import time
import copy
import numpy as np

def not_all_robots_at_goal(list_robots):
    store_result = []
    for robot in list_robots:
        store_result.append(robot.last_state[0] != robot.goal_state[0] or robot.last_state[1] != robot.goal_state[1])
    return np.sum(store_result)

def robot_not_at_goal(robot):
    return robot.last_state[0] != robot.goal_state[0] or robot.last_state[1] != robot.goal_state[1]

def get_current_go_to_state(robot):
    successors = robot.problem.getSuccessors(robot.last_state)
    robot.next_state = min(successors, key = lambda successor : robot.get_step_cost(successor[0]) + robot.graph_grid_world[successor[0][0]][successor[0][1]].g)[0]

def collision_checker(list_robots):
    for robot in list_robots:
        if len(robot.to_changed_coords) != 0:
            for coord in robot.to_changed_coords:
                if coord[1] == 1:
                    robot.to_changed_coords.remove(coord)
                else:
                    FLAG = 0
                    for frobot in list_robots:
                        if frobot.flag == 1:
                            print(coord[0])
                            if frobot.last_state[0] == coord[0][0][0] and frobot.last_state[1] == coord[0][0][1]:
                                robot.to_changed_coords.remove(coord)
                                FLAG = 1
                                break
                    if FLAG == 0:
                        coord[1] = 1
                        robot.FLAG += 1
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
            elif robot.flag == 1 and robot.last_state == sub_robot.next_state:
                if [[robot.last_state], 16] not in sub_robot.to_changed_coords:
                    sub_robot.to_changed_coords.append([[robot.last_state], 16])
                    sub_robot.FLAG += 1
            elif robot.next_state == sub_robot.next_state:
                if [[robot.next_state], 16] not in sub_robot.to_changed_coords:
                    sub_robot.to_changed_coords.append([[robot.next_state], 16])
                    sub_robot.FLAG += 1
            elif robot.next_state == sub_robot.last_state and robot.last_state == sub_robot.next_state:
                if [[robot.next_state], 16] not in sub_robot.to_changed_coords:
                    sub_robot.to_changed_coords.append([[robot.next_state], 16])
                    sub_robot.FLAG += 1
    for i in range(len(list_robots)):
        if list_robots[i].FLAG >= 1:
            list_robots[i].km += list_robots[i].h(list_robots[i].slast, list_robots[i].last_state)
            list_robots[i].slast = list_robots[i].last_state
            _ = list_robots[i].wait_for_changes(list_robots[i].to_changed_coords)
            list_robots[i].update_vertex(list_robots[i].last_state)
            list_robots[i].compute_shortest_path()
            get_current_go_to_state(list_robots[i])
        list_robots[i].FLAG = 0
      
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
            if not robot.flag:
                robot.charge -= 0.5
            else:
                if robot.charge < 100:
                    robot.charge += 0.5
    
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

def main():
    
    No_of_Robots = 5
    list_robots = [None for _ in range(No_of_Robots)]
    list_current_maze = [None for _ in range(No_of_Robots)]
    
    start_states = [(7, 8), (9, 6), (9, 9), (8, 11), (11, 8)]
    goal_states = [(1, 13), (1, 16), (4, 13), (4, 16), (6, 16)]
    
    #start_states = [[2, 3], [4, 4], [4, 3]]                                     # Start States
    #goal_states = [[3, 8], [4, 8], [5, 8]] 
    
    #start_states = [[3, 4], [4, 5], [4, 3]]                                     # Start States
    #goal_states = [[5, 4], [4, 3], [4, 5]] 
    
    final_goal_states = [None for _ in range(No_of_Robots)]
    for i in range(No_of_Robots):
        final_goal_states[i] = min(goal_states, key = lambda x : np.abs(x[0] - start_states[i][0]) + np.abs(x[1] - start_states[i][1]))
        goal_states.remove(final_goal_states[i])
       
    states = [[start_states[i], final_goal_states[i]] for i in range(No_of_Robots)]
    #states = [[start_states[i], goal_states[i]] for i in range(No_of_Robots)]
    
    for i in range(No_of_Robots):
        list_current_maze[i] = maze.Maze(1, states[i], i + 1)
        list_robots[i] = D_start_lite.DSTARLITE(list_current_maze[i], states, iden = i + 1)
    
    if sim_interface.sim_init():
        for i in range(len(list_robots)):
            list_robots[i].robot = sim_interface.Pioneer(list_robots[i].id)
            list_robots[i].robot.localize_robot()
            list_robots[i].old_goal = [list_robots[i].robot.current_state[0], list_robots[i].robot.current_state[1]]
    else:
        print ('Failed connecting to remote API server')
        
    for i in range(No_of_Robots):
        list_robots[i].initialize()
        list_robots[i].next_state = list_robots[i].start_state
        list_robots[i].last_state = list_robots[i].start_state
        list_robots[i].compute_shortest_path()
        
    while not_all_robots_at_goal(list_robots):
        for i in range(No_of_Robots):
            if list_robots[i].graph_grid_world[list_robots[i].last_state[0]][list_robots[i].last_state[1]].g == float('inf'):
                print(f"No known path for robot {i + 1}")
            else:
                if robot_not_at_goal(list_robots[i]):
                    get_current_go_to_state(list_robots[i])
        collision_checker(list_robots)
        move_to(list_robots)
        for i in range(No_of_Robots):
            list_robots[i].last_state = list_robots[i].next_state
        
        for i in range(No_of_Robots):
            if not robot_not_at_goal(list_robots[i]):
                list_robots[i].flag = 1
                for srobot in list_robots:
                    srobot.problem.map_plot_copy[list_robots[i].last_state[0]][list_robots[i].last_state[1]] = 16
        
        flags = list_robots[0].flag
        for i in range(1, len(list_robots)):
            flags = flags and list_robots[i].flag
        if flags:
            break
        
    time.sleep(5.0)
    sim_interface.sim_shutdown()
    time.sleep(2.0)
    
if __name__ == '__main__':

    main()                    
    print ('Program ended')
            

 
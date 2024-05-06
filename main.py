#!/usr/bin/env python

"""
Multi robot simulation setup
@author: Bijo Sebastian 
"""

#Import libraries
import time

#Import files
import sim_interface
import maze
import Adv_LPA_star_Final

def action_to_actuation(current_state, action):
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

def main():
    current_maze = maze.Maze(1)
    #path = search.weighted_AStarSearch(current_maze, 'M')
    lpastar = Adv_LPA_star_Final.LPASTAR(current_maze)
    path = lpastar.lpastar()
    print(path)
    if path:
        #Check path validity
        row,col = current_maze.getStartState() 
        for action in path:
            del_x, del_y = current_maze.four_neighbor_actions.get(action)
            row = row + del_x
            col = col + del_y
            if maze.enable_plots:
                #Update changes on the plot copy
                current_maze.map_plot_copy[row][col] = 10
        if maze.enable_plots:        
            #Plot the solution
            current_maze.plot_map()
        if current_maze.isGoalState([row, col]):
            print("Found a path of ", len(path)," moves by expanding ",current_maze.getStateExpansionCount()," nodes")
        else:
            print('Not a valid path')
        
    else:        
        print("Could not find a path")  
    if (sim_interface.sim_init()):
        #sim_interface.sim_shutdown()

        #Create three robot and setup interface for all three 
        robot1 = sim_interface.Pioneer(1)
        robot1.localize_robot()
        old_goal = [robot1.current_state[0], robot1.current_state[1]]
        #robot2 = sim_interface.Pioneer(2)
        #robot3 = sim_interface.Pioneer(3)

        #Start simulation
        if (sim_interface.start_simulation()):
            print(path)
            i = 1
            for action in path:
                print(i)
                robot1.localize_robot()
                robot1.goal_state = action_to_actuation(old_goal, action)
                
                while not robot1.robot_at_goal():
                #Run the control loops for three robots
                    robot1.run_controller()
                    print(robot1.current_state, robot1.goal_state)    
                old_goal = robot1.goal_state
                i += 1
        else:
            print ('Failed to start simulation')
    else:
        print ('Failed connecting to remote API server')
    #shutdown
    time.sleep(5.0)
    sim_interface.sim_shutdown()
    time.sleep(2.0)
    return

#run
if __name__ == '__main__':

    main()                    
    print ('Program ended')
            

 
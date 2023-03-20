import numpy as np
import time
import cv2
import math                              #### import libraries which are necessary
from queue import PriorityQueue


def move(node, action, L = 5):      ####function calls the next available node with an action and also specifies the robot orientation 
                                        ##along with the new position of robot
    x, y, theta = node
    theta = math.radians(theta)
    theta += math.radians(action)
    x += L * math.cos(theta)
    y += L * math.sin(theta)
    theta = math.degrees(theta) % 360
    return ((x//0.5)/2, (y//0.5)/2, theta), L


# Define obstacles (rectangles, hexagons, triangles) along with their respective half plane equations
def rectangle1(x, y):                                        # Define the rectangle function
    return (0 <= y <= 100 and 100 <= x <= 150)

def rectangle2(x, y):
    return 150 <= y <= 250 and 100 <= x <= 150

def hexagon(x, y):                                  # hexagon function specified
    return (75/2) * abs(x-300)/75 + 50 <= y <= 250 - (75/2) * abs(x-300)/75 - 50 and 225 <= x <= 375

def triangle(x, y):                     # triangle function specified
    return (200/100) * (x-460) + 25 <= y <= (-200/100) * (x-460) + 225 and 460 <= x <= 510

def border(x, y):                                       # border line function injuncted
    return y > 250 or y < 0 or x > 600 or x < 0

# Define the dictionary of obstacles (rectangles, hexagons, triangles) and their functions  
equation = [rectangle1, rectangle2, hexagon, triangle, border]
# getting start and goal nodes from the user
start_str = input("Enter the start node (format 'x y theta'): ")   ## input of start node
start_node = tuple(map(int, start_str.split()))
goal_str = input("\nEnter the goal node (format 'x y theta'): ")   ### input the goal node
goal_node = tuple(map(int, goal_str.split()))


clearance = int(input("\nEnter the clearance (in mm): "))   ####getting the user input clearance
radius = int(input("\nEnter the robot radius (in mm): "))   ####taking the radius of robot from user
stepsize = int(input("\nEnter the stepsize (in mm): "))     #### taking the stepsize of robot 

plot_width, plot_height, clearance = 600, 250, radius + clearance         ####goal desired dimensional units of the output image in pixels and set the width, height and clearance                  
pixels = np.full((plot_height, plot_width, 3), 255, dtype=np.uint8)       #### give out the plot height plot width, white color, and also dtype 


for i in range(plot_height):    ### going through every row that is the height
    for j in range(plot_width):  ### going through every column that is width
        for eqn in equation:
            if eqn(j, i):            ### if it happens go through each equation and print it black
                pixels[i, j] = [0, 0, 0]  
                break    ## break the cycle
            elif eqn(j, i-clearance) or eqn(j, i+clearance) or eqn(j-clearance, i) or eqn(j+clearance, i): ####if the user input is within the clearance ditance of the equation then paint it black
                pixels[i, j] = [192, 192, 192]  

def is_valid_node(node):                                                    ### x nad y should be within the plot area
    x, y, theta = node           
    x, y, theta = int(x), int(y), theta                                               # taking x and y from user
    return 0 <= x < plot_width and 0 <= y < plot_height and (pixels[y, x] == [255, 255, 255]).all()   ### pixel value mostly be white i.e the obstacle

def is_goal(current_node, goal_node):     #### the function takes in current goal and equatwes it with goal node
    return current_node == goal_node   #### now current node is taken as the goal node

def backtrack_path(parents, start_node, goal_node, animation):  ## defining in the parent dictionary , start and goal node as a list of nodes
    height, _, _ = animation.shape                                                                   
    path, current_node = [goal_node], goal_node   ## current node gets to goal node and is fixed
    while current_node != start_node:
        path.append(current_node)       ### the path is taken care of by continuosly appending the current node to the path
        current_node = parents[current_node]
        
        cv2.arrowedLine(animation, (int(current_node[0]), int(height - 1 - current_node[1])), (int(path[-1][0]), int(height - 1 - path[-1][1])), (0, 0, 255), 1)
        cv2.imshow('Animation', animation)
        cv2.waitKey(1)
    path.append(start_node)
    return path[::-1]

def astar(start_node, goal_node):    #### A star algorithm 
    open_list = PriorityQueue()       ### creating an empty priority queue
    closed_list = set()               ### to hold the nodes which are already explored
    cost_to_come = {start_node: 0}      ### creating dictionaries to find thge cost to cost
    cost = {start_node: 0}              ###adding the start node to open list
    parent = {start_node: None}          
    open_list.put((0, start_node))
    animation = pixels.copy()        ### copy of pixels is made
    visited = set([start_node])      ####  create the set to copy and track the explored nodes
    height, _, _ = animation.shape    ### height of the animation 
    while not open_list.empty():
        _, current_node = open_list.get()

        closed_list.add(current_node)    ### add the current node to closed list

        
        if parent[current_node] is not None:
            cv2.arrowedLine(animation, (int(current_node[0]), int(height - 1 - current_node[1])), (int(parent[current_node][0]), int(height - 1 - parent[current_node][1])), (0, 0, 255), 1)
        cv2.imshow('Animation', animation)   ### if it is the current node it draws an arrow to the parent node 
        cv2.waitKey(1)

        if is_goal(current_node, goal_node):        #### check if current node is the goal node
            path = backtrack_path(parent, start_node, goal_node, animation)
            # Mark start and goal nodes as green
            animation[int(height - 1 - start_node[1]), int(start_node[0])] = (0, 255, 0)    ### start and goal is coloured to green in animation
            animation[int(height - 1 - goal_node[1]), int(goal_node[0])] = (0, 255, 0)
            cv2.imshow('Animation', animation)           #give out the animation
            cv2.waitKey(0)
            cv2.destroyAllWindows()
           
            print("Final Cost: ", cost[goal_node])      #### print the final cost
            return path

        for action in [-60, -30, 0, 30, 60]:
            new_node, move_cost = move(current_node, action, stepsize)      ### calculating the new and unexplored nodes along with the cost

            if is_valid_node(new_node) and new_node not in closed_list:               ### adding the current cost to move cost
                new_cost_to_come = cost_to_come[current_node] + move_cost
                new_cost = new_cost_to_come + 5 * heuristic(new_node, goal_node)     ####The function modifies the cost to come, cost, and value variables if the current cost to come is less than the previous cost to come.

                if new_node not in cost_to_come or new_cost_to_come < cost_to_come[new_node]:
                    cost_to_come[new_node] = new_cost_to_come
                    cost[new_node] = new_cost        #### adding the new cost of new node to new cost
                    parent[new_node] = current_node      #### add new node to current node
                    open_list.put((new_cost, new_node))
                    visited.add(new_node)
    cv2.imshow('Animation', animation)     ### put out the animation
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return None



def heuristic(node, goal_node):   #### calculates the distance between the given node and goal node
    return abs(goal_node[0] - node[0]) + abs(goal_node[1] - node[1])


if not is_valid_node(start_node):   #### checks whether the given start node is valid or not
    print(f"Invalid {start_str} start node. Give a valid node i.e. not in the obstacle space.")  ### if not prints this
elif not is_valid_node(goal_node):  ### checks whether the goal input is goal node
    print(f"Invalid {goal_str} goal node. Give a valid node i.e. not in the obstacle space.")   ### if not porints this
else:
   
   
    start_time = time.time()    ### measure the time taken form start
    path = astar(start_node, goal_node)    #### path along the given start and goal nodes
    if path is None:
        print("Path Not Found")
    else:
        print("Goal Node Achieved")
    end_time = time.time()
    print(f"Time taken to execute A* Algorithm: {time.time() - start_time} seconds")     ### gives out time taken to execute

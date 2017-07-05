#!/usr/bin/python

import numpy as np
import yaml
import math
from operator import itemgetter
import heapq
import pprint




def dijkstras(occupancy_map, x_spacing, y_spacing, start, goal):
    """
    Implements Dijkstra's shortest path algorithm
    Input:
    occupancy_map - an N by M numpy array of boolean values (represented
        as integers 0 and 1) that represents the locations of the obstacles
        in the world
    x_spacing - parameter representing spacing between adjacent columns
    y_spacing - parameter representing spacing between adjacent rows
    start - a 3 by 1 numpy array of (x,y,theta) for the starting position 
    goal - a 3 by 1 numpy array of (x,y,theta) for the finishing position 
    Output: 
    path: list of the indices of the nodes on the shortest path found
        starting with "start" and ending with "end" (each node is in
        metric coordinates)
    """
    # We will use this delta function to search surrounding nodes.
    delta = [[-1, 0],  # go up
             [0, -1],  # go left
             [1, 0],  # go down
             [0, 1]]  # go right

    # Each node on the map "costs" 1 step to reach.
    cost = 1
    # Convert numpy array of map to list of map, makes it easier to search.
    occ_map = occupancy_map.tolist()

    # Converge start and goal positions to map indices.
    x = int(math.ceil((start.item(0) / x_spacing) - 0.5))  # startingx
    y = int(math.ceil((start.item(1) / y_spacing) - 0.5))  # startingy
    goalX = int(math.ceil((goal.item(0) / x_spacing) - 0.5))
    goalY = int(math.ceil((goal.item(1) / y_spacing) - 0.5))
    print "Start Pose: ", x, y
    print "Goal Pose: ", goalX, goalY

    # Make a map to keep track of all the nodes and their cost distance values.
    possible_nodes = [[0 for row in range(len(occ_map[0]))] for col in range(len(occ_map[1]))]
    row = y
    col = x

    possible_nodes[row][col] = 1 #This means the starting node has been searched.
    print "Possible Nodes: "
    pprint.pprint(possible_nodes)

    # The g_value will count the number of steps each node is from the start.
    # Since we are at the start node, the total cost is 0.
    g_value = 0
    open_nodes = [(g_value, col, row)] # dist, x, y
    searched_nodes = []
    loopcount = 0

    while len(open_nodes) != 0:
        print "\n>>>>>>>>>>>>LOOP COUNT: ", loopcount, "\n"
        open_nodes.sort(reverse=True) #sort from shortest distance to farthest
        nearest_node = open_nodes.pop()
        print "meow: ", nearest_node
        heapq.heappush(searched_nodes, nearest_node)
        print "mooo: ", searched_nodes
        if nearest_node[1] == goalX and nearest_node[2] == goalY:
            print "Goal found!"
            print "NEAREST NODE: ", nearest_node
            print "searched_nodes: \n", searched_nodes
            print "\n"
            print sorted(searched_nodes, key = itemgetter(0))
            break
        g_value, col, row = nearest_node
        print "current g, col, row:", g_value, col, row
        for i in delta:
            possible_expansion_x = col + i[0]
            possible_expansion_y = row + i[1]
            valid_expansion = 0 <= possible_expansion_x < len(occupancy_map[0]) and 0 <= possible_expansion_y < len(occ_map)

            if valid_expansion:
                unsearched_node = possible_nodes[possible_expansion_x][possible_expansion_y] == 0
                open_node = occ_map[possible_expansion_x][possible_expansion_y] == 0
                if unsearched_node and open_node:
                    possible_nodes[possible_expansion_x][possible_expansion_y] = 1
                    open_nodes.append((g_value + cost, possible_expansion_x, possible_expansion_y))
                    print "added_nodes:", open_nodes
                    print "While Possible Nodes: "
                    pprint.pprint(possible_nodes)
        loopcount = loopcount+1

    print "Generating path..."

    path = []
    position = [start.item(0), start.item(1)] #starting point passed in by function
    path.append(position) #add it to the list for the path

    print "Pathhhhh: ", path
    print "Pop POP: "
    print heapq.heappop(searched_nodes)
    print heapq.heappop(searched_nodes)
    print heapq.heappop(searched_nodes)
    print heapq.heappop(searched_nodes)
    print heapq.heappop(searched_nodes)
    print heapq.heappop(searched_nodes)
    print heapq.heappop(searched_nodes)
    # position = [(x+0.5)*x_spacing, (y+0.5)*y_spacing]
    # path.append(position)


def test():
    """
    Function that provides a few examples of maps and their solution paths
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 0.13
    y_spacing1 = 0.2
    start1 = np.array([[0.3], [0.3], [0]])
    goal1 = np.array([[0.6], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    true_path1 = np.array([
        [ 0.3  ,  0.3  ], # [2,1]
        [ 0.325,  0.3  ], # [2,1]
        [ 0.325,  0.5  ], # [2,2]
        [ 0.325,  0.7  ], # [2,3]
        [ 0.455,  0.7  ], # [3,3]
        [ 0.455,  0.9  ], # [3,4]
        [ 0.585,  0.9  ], # [4,4]
        [ 0.600,  1.0  ]  # [5,5]
        ])
    if np.array_equal(path1,true_path1):
      print("Path 1 passes")
    else:
        print "nope, chuck testa"

    # test_map2 = np.array([
    #          [0, 0, 0, 0, 0, 0, 0, 0],
    #          [0, 0, 0, 0, 0, 0, 0, 0],
    #          [0, 0, 0, 0, 0, 0, 0, 0],
    #          [1, 1, 1, 1, 1, 1, 1, 1],
    #          [1, 0, 0, 1, 1, 0, 0, 1],
    #          [1, 0, 0, 1, 1, 0, 0, 1],
    #          [1, 0, 0, 1, 1, 0, 0, 1],
    #          [1, 0, 0, 0, 0, 0, 0, 1],
    #          [1, 0, 0, 0, 0, 0, 0, 1],
    #          [1, 1, 1, 1, 1, 1, 1, 1]])
    # start2 = np.array([[0.5], [1.0], [1.5707963267948966]])
    # goal2 = np.array([[1.1], [0.9], [-1.5707963267948966]])
    # x_spacing2 = 0.2
    # y_spacing2 = 0.2
    # path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    # true_path2 = np.array([[ 0.5,  1.0],
    #                        [ 0.5,  1.1],
    #                        [ 0.5,  1.3],
    #                        [ 0.5,  1.5],
    #                        [ 0.7,  1.5],
    #                        [ 0.9,  1.5],
    #                        [ 1.1,  1.5],
    #                        [ 1.1,  1.3],
    #                        [ 1.1,  1.1],
    #                        [ 1.1,  0.9]])
    # if np.array_equal(path2,true_path2):
    #   print("Path 2 passes")

def test_for_grader():
    """
    Function that provides the test paths for submission
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 1, 0, 0, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 0, 0, 1, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 1
    y_spacing1 = 1
    start1 = np.array([[1.5], [1.5], [0]])
    goal1 = np.array([[7.5], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    s = 0
    for i in range(len(path1)-1):
      s += np.sqrt((path1[i][0]-path1[i+1][0])**2 + (path1[i][1]-path1[i+1][1])**2)
    print("Path 1 length:")
    print(s)


    test_map2 = np.array([
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.4], [0.4], [1.5707963267948966]])
    goal2 = np.array([[0.4], [1.8], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    s = 0
    for i in range(len(path2)-1):
      s += np.sqrt((path2[i][0]-path2[i+1][0])**2 + (path2[i][1]-path2[i+1][1])**2)
    print("Path 2 length:")
    print(s)



def main():
    test()
    # # Load parameters from yaml
    # param_path = 'params.yaml' # rospy.get_param("~param_path")
    # f = open(param_path,'r')
    # params_raw = f.read()
    # f.close()
    # params = yaml.load(params_raw)
    # # Get params we need
    # occupancy_map = np.array(params['occupancy_map'])
    # pos_init = np.array(params['pos_init'])
    # pos_goal = np.array(params['pos_goal'])
    # x_spacing = params['x_spacing']
    # y_spacing = params['y_spacing']
    # path = dijkstras(occupancy_map,x_spacing,y_spacing,pos_init,pos_goal)
    # print(path)

if __name__ == '__main__':
    main()


#!/usr/bin/python

import numpy as np
import yaml
import math



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
    path = []
    validpath = False #found
    nopath = False #resign
    step = 1 #how far each step is in the grid representation. Cost.
    delta = [[-1, 0],  # go up
             [0, -1],  # go left
             [1, 0],  # go down
             [0, 1]]  # go right

    g = 0 #this is how many grid "steps" away each node is from the start node.
    x = int(math.ceil((start.item(0)/x_spacing) - 0.5)) #startingx
    y = int(math.ceil((start.item(1)/y_spacing) - 0.5)) #startingy
    goalX = int(math.ceil((goal.item(0)/x_spacing)-0.5))
    goalY = int(math.ceil((goal.item(1)/y_spacing)-0.5))
    open_nodes = [[step, x, y]] #This is a list that captures each of the initial
    print "x, y", x, y, type(x), type(y)

    position = [start[0], start[0]] #starting point passed in by function
    path.append(position) #add it to the list for the path
    position = [(x+0.5)*x_spacing, (y+0.5)*y_spacing]
    path.append(position)

    # print "First entry: ", path[1]
    # print "Map location: ", startx, starty
    # print len(occupancy_map[1,:])
    # print len(occupancy_map[:,1])

    #closed in example
    searched = [[0 for row in range(len(occupancy_map[1,:]))] for col in range(len(occupancy_map[:,1]))]
    searched[y][x] = 1 #remember, j = rows, i = colomns



    while validpath == False  and nopath ==False:
        if len(open_nodes) == 0: #empty, nothing to search through
            nopath = True
            print "No valid path found..."
            path = np.zeros(shape=(1, 2))  # return back a zero path
            return path

        else:
            #remove node from list
            open_nodes.sort(reverse=True) #Sorts from lowest to highest values of g
            next = open_nodes.pop()
            print 'take list item'
            print next
            x = next[1]
            y = next[2]
            g = next[0]



            #check if goal value:

            if x == goalX and y == goalY:
                validpath = True
                print "Valid path found!"
                position = [(x + 0.5) * x_spacing, (y + 0.5) * y_spacing]
                path.append(position)
                path = np.array(path)
                return path

            else:
                #if not the goal...continue expanding out in every direction
                for i in range(len(moves)):
                    print "moves: ", moves[0][i]
                    x2 = x + moves[0][i]
                    y2 = y + moves[i][1]

                    if x2 >=0 and x2 < len(searched[0]) and y2 >= 0 and y2 < len(searched):
                        if searched[x2][y2] == 0 and occupancy_map[y2][x2] == 0:
                            g2 = g + step
                            open_nodes.append([g2,x2,y2])
                            searched[x2][y2] = 1










    # pass

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
        [ 0.3  ,  0.3  ],
        [ 0.325,  0.3  ],
        [ 0.325,  0.5  ],
        [ 0.325,  0.7  ],
        [ 0.455,  0.7  ],
        [ 0.455,  0.9  ],
        [ 0.585,  0.9  ],
        [ 0.600,  1.0  ]
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


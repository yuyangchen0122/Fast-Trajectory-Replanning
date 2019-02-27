import numpy as np
from ezgraphics import GraphicsWindow
import sys
import heapq
from ClassForAdaptive import *
import time
import random

maze_size = 101

starting_x = random.randint(0, 100)
starting_y = random.randint(0, 100)

target_x = random.randint(0, 100)
target_y = random.randint(0, 100)

# initialize the information matrix
def setup_node():
    grid = makeGrid()
    grid[starting_x][starting_y] = node()
    origin = grid[starting_x][starting_y]
    origin.x = starting_x
    origin.y = starting_y

    origin.h = np.abs(target_x - starting_x) + np.abs(target_y - starting_y)
    origin.g = 0

    grid[target_x][target_y] = node()
    destination = grid[target_x][target_y]
    destination.x = target_x
    destination.y = target_y
    destination.h = 0
    destination.g = sys.maxsize
    return grid


def isValid(x, y):
    return ((x >= 0) and (x < maze_size) and (y >= 0) and (y < maze_size))


def update_neighbour_node_A(Maze, Mazeinfor, current_node, s_goal, Queue, open_closed_list, visited_list):
    global Adaptive
    current_x = current_node.x
    current_y = current_node.y



    # update right neighbor_node

    if (isValid(current_x + 1, current_y) and (not open_closed_list[current_x + 1][current_y])):
        open_closed_list[current_x + 1][current_y] = True
        if (not isinstance(Mazeinfor[current_x + 1][current_y], node)):
            Mazeinfor[current_x + 1][current_y] = node()
        neighbor_node = Mazeinfor[current_x + 1][current_y]
        if (not neighbor_node.isBlocked):
            neighbor_node.parent = current_node
            neighbor_node.g = current_node.g + 1
            neighbor_node.x = current_x + 1
            neighbor_node.y = current_y
            if (neighbor_node.nh == -1):
                neighbor_node.h = Manhattan(neighbor_node, s_goal)
            else:
                neighbor_node.h = neighbor_node.nh
            MinHeap.push(Queue, neighbor_node)
            Adaptive += 1
            visited_list.append(neighbor_node)
    # print("push point {} {}".format(current_x + 1, current_y))

    # update left neighbor_node
    if (isValid(current_x - 1, current_y) and (not open_closed_list[current_x - 1][current_y])):
        open_closed_list[current_x - 1][current_y] = True
        # this is equal to check if succ(s, a) < counter
        if (not isinstance(Mazeinfor[current_x - 1][current_y], node)):
            Mazeinfor[current_x - 1][current_y] = node()
        neighbor_node = Mazeinfor[current_x - 1][current_y]
        if (not neighbor_node.isBlocked):
            neighbor_node.parent = current_node
            neighbor_node.g = current_node.g + 1
            neighbor_node.x = current_x - 1
            neighbor_node.y = current_y
            if (neighbor_node.nh == -1):
                neighbor_node.h = Manhattan(neighbor_node, s_goal)
            else:
                neighbor_node.h = neighbor_node.nh
            MinHeap.push(Queue, neighbor_node)
            Adaptive += 1
            visited_list.append(neighbor_node)

    # print("push point {} {}".format(current_x - 1, current_y))

    # update downward neighbor_node
    if (isValid(current_y, current_y - 1) and (not open_closed_list[current_x][current_y - 1])):
        open_closed_list[current_y][current_y - 1] = True
        if (not isinstance(Mazeinfor[current_y][current_y - 1], node)):
            Mazeinfor[current_y][current_y - 1] = node()
        neighbor_node = Mazeinfor[current_y][current_y - 1]
        if (not neighbor_node.isBlocked):
            neighbor_node.parent = current_node
            neighbor_node.g = current_node.g + 1
            neighbor_node.x = current_y
            neighbor_node.y = current_y - 1
            if (neighbor_node.nh == -1):
                neighbor_node.h = Manhattan(neighbor_node, s_goal)
            else:
                neighbor_node.h = neighbor_node.nh
            MinHeap.push(Queue, neighbor_node)
            Adaptive += 1
            visited_list.append(neighbor_node)
    # print("push point {} {}".format(current_x, current_y - 1))

    # update upward neighbor_node
    if (isValid(current_y, current_y + 1) and (not open_closed_list[current_y][current_y + 1])):
        open_closed_list[current_y][current_y + 1] = True
        if (not isinstance(Mazeinfor[current_y][current_y + 1], node)):
            Mazeinfor[current_y][current_y + 1] = node()
        neighbor_node = Mazeinfor[current_y][current_y + 1]
        if (not neighbor_node.isBlocked):
            neighbor_node.parent = current_node
            neighbor_node.g = current_node.g + 1
            neighbor_node.x = current_y
            neighbor_node.y = current_y + 1
            if (neighbor_node.nh == -1):
                neighbor_node.h = Manhattan(neighbor_node, s_goal)
            else:
                neighbor_node.h = neighbor_node.nh
            MinHeap.push(Queue, neighbor_node)
            Adaptive += 1
            visited_list.append(neighbor_node)
    # print("push point {} {}".format(current_x, current_y + 1))
    return


# Make the grid, having the random origin and destination block set to unblocked and seen
def setup():
    grid = makeGrid()
    for i in range(maze_size):
        for j in range(maze_size):
            # Initialize each object
            if (starting_x and j == starting_y) or (i == target_x and j == target_y):
                # Cell(x coor, y coor, if_blocked, if_visited)
                grid[i][j] = Cell(i, j, False, True)
            else:
                grid[i][j] = Cell(i, j, randomization())
    return grid


def detect(s, maze, Mazeinfor):
    current_x = s.x
    current_y = s.y
    # print("locate at [{} {}]".format(current_x, current_y))
    if (isValid(current_x - 1, current_y)):
        # this is equal to check if succ(s, a) < counter
        if (not isinstance(Mazeinfor[current_x - 1][current_y], node)):
            Mazeinfor[current_x - 1][current_y] = node()
        neighbor_node = Mazeinfor[current_x - 1][current_y]
        neighbor_node.isBlocked = maze[current_x - 1][current_y].ifBlocked

    if (isValid(current_x + 1, current_y)):
        if (not isinstance(Mazeinfor[current_x + 1][current_y], node)):
            Mazeinfor[current_x + 1][current_y] = node()
        neighbor_node = Mazeinfor[current_x + 1][current_y]
        neighbor_node.isBlocked = maze[current_x + 1][current_y].ifBlocked

    if (isValid(current_x, current_y - 1)):
        if (not isinstance(Mazeinfor[current_x][current_y - 1], node)):
            Mazeinfor[current_x][current_y - 1] = node()
        neighbor_node = Mazeinfor[current_x][current_y - 1]
        neighbor_node.isBlocked = maze[current_x][current_y - 1].ifBlocked

    if (isValid(current_x, current_y + 1)):
        if (not isinstance(Mazeinfor[current_x][current_y + 1], node)):
            Mazeinfor[current_x][current_y + 1] = node()
        neighbor_node = Mazeinfor[current_x][current_y + 1]
        neighbor_node.isBlocked = maze[current_x][current_y + 1].ifBlocked

    return


# Return false for unblocked, true for blocked
def randomization():
    temp = np.random.choice([0, 1], 1, p=[0.3, 0.7])
    if temp[0] == 1:
        return False
    return True


def makeGrid():
    grid = [[0 for x in range(maze_size)] for y in range(maze_size)]
    return grid


def Manhattan(origin, goal):
    return (abs(goal.x - origin.x) + abs(goal.y - origin.y))


def ComputePath_A(Maze, Mazeinfor, s_goal, Queue, open_closed_list, visited_list):
    global g_goal
    # check whether queue is empty
    while (len(Queue) > 0):
        # print(len(Queue))
        '''
        for i in range(len(Queue)):
            n = Queue._MinHeap__heap[i]
            print("[{} {} {} {}] ".format(n.x, n.y, n.g, n.g + n.h), end = "")
        print(" ")
        print(" ")
        '''
        current_node = MinHeap.pop(Queue)
        # print("pop point {} {}".format(current_node.x, current_node.y))
        current_x = current_node.x
        current_y = current_node.y
        open_closed_list[current_x][current_y] = True
        if (current_node.x == s_goal.x and current_node.y == s_goal.y):
            g_goal = current_node.g
            return
        # update s's neighbor_nodes, executing step 5 to 13
        update_neighbour_node_A(Maze, Mazeinfor, current_node, s_goal, Queue, open_closed_list, visited_list)


def traceback(map_node_info, s_goal):
    tracklist = node()
    ptr = s_goal
    # while ptr hasn't reach the origin node
    while (ptr.g != 0):
        tracklist.add_front(ptr)
        ptr = ptr.parent

    tracklist.add_front(ptr)
    return tracklist.next


'''
traceback function serves to record the current ideal path that the agent 
estimate from the current position to the destination
In the form of a linked list
'''


def final_trace(map_node_info, s_goal):
    tracklist = node()
    ptr = s_goal
    # while ptr hasn't reach the origin node
    while (not (ptr.x == 0 and ptr.y == 0)):
        tracklist.add_front(ptr)
        ptr = ptr.parent
    # print("ptr is [{} {}]".format(ptr.x, ptr.y))

    tracklist.add_front(ptr)
    return tracklist.next


def take_action(track, maze, map_node_info, path):
    x = track.x
    y = track.y
    # print("check position [{} {}]".format(x, y))
    position = None
    if (map_node_info[x][y].g != 0):
        print("wrong origin point")
        exit(0)
    else:
        # keep moving until
        while (track != None):
            x = track.x
            y = track.y
            if (not map_node_info[x][y].isBlocked):
                detect(map_node_info[x][y], maze, map_node_info)
                position = track
                path.push(position.x, position.y)
                track = track.next
            else:
                break
    # need to complete
    return position


def main_A():
    global g_goal
    global Adaptive
    origin = time.time()
    # generate a random foggy map
    maze = setup()
    # generate a information map
    map_node_info = setup_node()

    # start from the begining, end at the goal stage
    s_start = map_node_info[starting_x][starting_y]
    print("Starting node is : " + str(starting_x) + ", " + str(starting_y))
    # detect the block
    detect(s_start, maze, map_node_info)
    s_goal = map_node_info[target_x][target_y]
    print("Ending node is : " + str(target_x) + ", " + str(target_y))

    s_start.nh = Manhattan(s_start, s_goal)
    path = point(-1, -1)
    while not (s_start.x == s_goal.x and s_start.y == s_goal.y):
        visited_list = []
        openlist = MinHeap()
        open_closed_list = [[False for i in range(maze_size)] for j in range(maze_size)]
        s_start.g = 0
        # push the start stage information to queue
        s_start.h = s_start.nh
        MinHeap.push(openlist, s_start)
        Adaptive += 1
        visited_list.append(s_start)
        # print("push point {} {}".format(s_start.x, s_start.y))

        '''
        track record the current idea path from current start goal to the final goal
        '''
        ComputePath_A(maze, map_node_info, s_goal, openlist, open_closed_list, visited_list)
        '''
        update the hnew value
        '''
        for i in visited_list:
            i.nh = g_goal - i.g
        track = traceback(map_node_info, s_goal)
        if len(openlist) == 0:
            print("I cannot reach the target.")
            return

        '''
        while(ptr != None):
            print('track is [{} {}]'.format(ptr.x, ptr.y), end=' ')
            ptr = ptr.next
        '''
        s_start = take_action(track, maze, map_node_info, path)
        # print("move to point [{} {}]".format(s_start.x, s_start.y))
        # print("current path end is [{} {}]".format(s_start.x, s_start.y))
    # print("goal point is [{} {}]".format(s_goal.x, s_goal.y))

    '''
        follow the tree pointers from s_goal to s_start, use a linkedlist to record
        the path, and then move the agent to the goal stage
    '''
    # final_track = final_trace(map_node_info, s_goal)
    ptr = path.next
    '''
    while ptr != None:
        print("path is [{} {}]".format(ptr.x, ptr.y), end = " ")
        ptr = ptr.next
    '''
    destination = time.time()
    print(destination - origin)

    return


def ComputePath_F(Maze, Mazeinfor, s_goal, Queue, open_closed_list):
    # check whether queue is empty
    while (len(Queue) > 0):
        # print(len(Queue))
        '''
        for i in range(len(Queue)):
            n = Queue._MinHeap__heap[i]
            print("[{} {} {} {}] ".format(n.x, n.y, n.g, n.g + n.h), end = "")
        print(" ")
        print(" ")
        '''
        current_node = MinHeap.pop(Queue)
        # print("pop point {} {}".format(current_node.x, current_node.y))
        current_x = current_node.x
        current_y = current_node.y
        open_closed_list[current_x][current_y] = True
        if (current_node.x == s_goal.x and current_node.y == s_goal.y):
            return
        # update s's neighbor_nodes, executing step 5 to 13
        update_neighbour_node_F(Maze, Mazeinfor, current_node, s_goal, Queue, open_closed_list)


def update_neighbour_node_F(Maze, Mazeinfor, current_node, s_goal, Queue, open_closed_list):
    global Forward
    current_x = current_node.x
    current_y = current_node.y

    # update right neighbor_node

    if (isValid(current_x + 1, current_y) and (not open_closed_list[current_x + 1][current_y])):
        open_closed_list[current_x + 1][current_y] = True
        if (not isinstance(Mazeinfor[current_x + 1][current_y], node)):
            Mazeinfor[current_x + 1][current_y] = node()
        neighbor_node = Mazeinfor[current_x + 1][current_y]
        if (not neighbor_node.isBlocked):
            neighbor_node.parent = current_node
            neighbor_node.g = current_node.g + 1
            neighbor_node.x = current_x + 1
            neighbor_node.y = current_y
            neighbor_node.h = Manhattan(neighbor_node, s_goal)
            MinHeap.push(Queue, neighbor_node)
            Forward += 1
    # print("push point {} {}".format(current_x + 1, current_y))

    # update left neighbor_node
    if (isValid(current_x - 1, current_y) and (not open_closed_list[current_x - 1][current_y])):
        open_closed_list[current_x - 1][current_y] = True
        # this is equal to check if succ(s, a) < counter
        if (not isinstance(Mazeinfor[current_x - 1][current_y], node)):
            Mazeinfor[current_x - 1][current_y] = node()
        neighbor_node = Mazeinfor[current_x - 1][current_y]
        if (not neighbor_node.isBlocked):
            neighbor_node.parent = current_node
            neighbor_node.g = current_node.g + 1
            neighbor_node.x = current_x - 1
            neighbor_node.y = current_y
            neighbor_node.h = Manhattan(neighbor_node, s_goal)
            MinHeap.push(Queue, neighbor_node)
            Forward += 1

    # print("push point {} {}".format(current_x - 1, current_y))

    # update downward neighbor_node
    if (isValid(current_x, current_y - 1) and (not open_closed_list[current_x][current_y - 1])):
        open_closed_list[current_x][current_y - 1] = True
        if (not isinstance(Mazeinfor[current_x][current_y - 1], node)):
            Mazeinfor[current_x][current_y - 1] = node()
        neighbor_node = Mazeinfor[current_x][current_y - 1]
        if (not neighbor_node.isBlocked):
            neighbor_node.parent = current_node
            neighbor_node.g = current_node.g + 1
            neighbor_node.x = current_x
            neighbor_node.y = current_y - 1
            neighbor_node.h = Manhattan(neighbor_node, s_goal)
            MinHeap.push(Queue, neighbor_node)
            Forward += 1
    # print("push point {} {}".format(current_x, current_y - 1))

    # update upward neighbor_node
    if (isValid(current_x, current_y + 1) and (not open_closed_list[current_x][current_y + 1])):
        open_closed_list[current_x][current_y + 1] = True
        if (not isinstance(Mazeinfor[current_x][current_y + 1], node)):
            Mazeinfor[current_x][current_y + 1] = node()
        neighbor_node = Mazeinfor[current_x][current_y + 1]
        if (not neighbor_node.isBlocked):
            neighbor_node.parent = current_node
            neighbor_node.g = current_node.g + 1
            neighbor_node.x = current_x
            neighbor_node.y = current_y + 1
            neighbor_node.h = Manhattan(neighbor_node, s_goal)
            MinHeap.push(Queue, neighbor_node)
            Forward += 1
    # print("push point {} {}".format(current_x, current_y + 1))
    return


def main_F():
    global Forward
    origin = time.time()
    # generate a random foggy map
    maze = setup()
    # generate a information map
    map_node_info = setup_node()

    # start from the begining, end at the goal stage
    s_start = map_node_info[starting_x][starting_y]

    # detect the block
    detect(s_start, maze, map_node_info)
    s_goal = map_node_info[target_x][target_y]
    path = point(-1, -1)
    while not (s_start.x == s_goal.x and s_start.y == s_goal.y):
        openlist = MinHeap()
        open_closed_list = [[False for i in range(maze_size)] for j in range(maze_size)]
        s_start.g = 0
        # push the start stage information to queue
        s_start.h = Manhattan(s_start, s_goal)
        MinHeap.push(openlist, s_start)
        Forward += 1
        # print("push point {} {}".format(s_start.x, s_start.y))

        '''
        track record the current idea path from current start goal to the final goal
        '''
        ComputePath_F(maze, map_node_info, s_goal, openlist, open_closed_list)
        track = traceback(map_node_info, s_goal)
        if len(openlist) == 0:
            print("I cannot reach the target.")
            return

        '''
        while(ptr != None):
            print('track is [{} {}]'.format(ptr.x, ptr.y), end=' ')
            ptr = ptr.next
        '''
        s_start = take_action(track, maze, map_node_info, path)
        # print("move to point [{} {}]".format(s_start.x, s_start.y))
        # print("current path end is [{} {}]".format(s_start.x, s_start.y))
    # print("goal point is [{} {}]".format(s_goal.x, s_goal.y))

    '''
        follow the tree pointers from s_goal to s_start, use a linkedlist to record
        the path, and then move the agent to the goal stage
    '''
    # final_track = final_trace(map_node_info, s_goal)
    ptr = path.next
    '''
    while ptr != None:
        print("path is [{} {}]".format(ptr.x, ptr.y), end = " ")
        ptr = ptr.next
    '''
    destination = time.time()
    print("Time:", destination - origin)

    return


if __name__ == "__main__":
    g_goal = 0
    Forward = 0
    Adaptive = 0
    main_F()
    main_A()
    print("Forward is {}".format(Forward))
    print("Adaptive is {}".format(Adaptive))
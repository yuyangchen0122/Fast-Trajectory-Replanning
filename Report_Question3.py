import numpy as np
from ezgraphics import GraphicsWindow
import sys
from ClassForRepeated import *
import time
import gc
import random
import csv
import collections

maze_size = 101

starting_x = random.randint(0, 100)
starting_y = random.randint(0, 100)

target_x = random.randint(0, 100)
target_y = random.randint(0, 100)

starting_node_string = 0


class time1:
    average_time = 0
    valid_count = 0


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


def update_neighbour_node(Maze, Mazeinfor, current_node, s_goal, Queue, open_closed_list):
    current_x = current_node.x
    current_y = current_node.y
    global counter
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
            counter += 1
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
            counter += 1

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
            counter += 1
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
            counter += 1
    # print("push point {} {}".format(current_x, current_y + 1))
    return


# Make the grid, having the random origin and destination block set to unblocked and seen
def setup():
    grid = makeGrid()
    for i in range(maze_size):
        for j in range(maze_size):
            # Initialize each object
            if (i == starting_x and j == starting_y) or (i == target_x and j == target_y):
                # Cell(x coor, y coor, if_blocked, if_visited)
                grid[i][j] = Cell(i, j, False, True)
            else:
                grid[i][j] = Cell(i, j, random_blocked())

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
def random_blocked():
    temp = np.random.choice([0, 1], 1, p=[0.3, 0.7])
    if temp[0] == 1:
        return False
    return True


# making a grid as [101][101]
def makeGrid():
    grid = [[0 for x in range(maze_size)] for y in range(maze_size)]
    return grid


def draw(maze, path_list, off=7):
    win = GraphicsWindow(maze_size * off * 1.2, maze_size * off * 1.2)
    canvas = win.canvas()
    cell_size = off  # Height and width of checkerboard squares.
    # start

    for i in range(maze_size):  # Note that i ranges from 0 through 7, inclusive.
        for j in range(maze_size):  # So does j.
            cell = maze[i][j]
            if not cell.ifBlocked:
                color = 'white'
            else:
                color = 'black'

            canvas.setFill(color)
            # draw cell_size * cell_size rectangle at point (offset_x + i * cell_size, offset_y + j * cell_size)
            canvas.drawRect(off + i * cell_size, off + j * cell_size, cell_size, cell_size)

    ptr = path_list.next
    while (ptr.next != None):
        ptr = ptr.next

    while (ptr != None):
        current_x = ptr.x
        current_y = ptr.y
        canvas.setFill('red')

        canvas.drawOval(off + current_x * cell_size + cell_size * .2, off + current_y * cell_size + cell_size * .2,
                        cell_size * .6, cell_size * .6)

        # canvas.drawRect(off + current_x * cell_size, off + current_y * cell_size, cell_size, cell_size)
        # print("path at [{} {}]".format(current_x, current_y))
        ptr = ptr.parent

    win.wait()


def Manhattan(origin, goal):
    return (abs(goal.x - origin.x) + abs(goal.y - origin.y))


def ComputePath(Maze, Mazeinfor, s_goal, Queue, open_closed_list):
    # check whether queue is empty
    while (len(Queue) > 0):

        current_node = MinHeap.pop(Queue)
        # print("pop point {} {}".format(current_node.x, current_node.y))
        current_x = current_node.x
        current_y = current_node.y
        open_closed_list[current_x][current_y] = True
        if (current_node.x == s_goal.x and current_node.y == s_goal.y):
            return
        # update s's neighbor_nodes, executing step 5 to 13
        update_neighbour_node(Maze, Mazeinfor, current_node, s_goal, Queue, open_closed_list)


def traceback(map_node_info, s_goal):
    tracklist = node()
    ptr = s_goal
    # while ptr hasn't reach the start node
    while (ptr.g != 0):
        tracklist.add_front(ptr)
        if (ptr.parent == None):
            print(ptr.x, ptr.y)
            print("can't get path")
            break
        else:
            ptr = ptr.parent

    tracklist.add_front(ptr)
    return tracklist.next


def final_trace(map_node_info, s_goal):
    tracklist = node()
    ptr = s_goal
    # while ptr hasn't reach the start node
    while (not (ptr.x == starting_x and ptr.y == starting_y)):
        tracklist.add_front(ptr)
        ptr = ptr.parent
    # print("ptr is [{} {}]".format(ptr.x, ptr.y))

    tracklist.add_front(ptr)
    return tracklist.next


def take_action_Forward(track, maze, map_node_info, path):
    global counter
    x = track.x
    y = track.y
    # print("check position [{} {}]".format(x, y))
    position = None
    if (map_node_info[x][y].g != 0):
        print("wrong start point")
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
                counter += 1
                track = track.next
            else:
                break
    # need to complete
    return position


def main_Forward():
    global counter
    origin = time.time()
    # generate a random foggy map
    maze = setup()
    # generate a information map
    map_node_info = setup_node()

    counter = 0
    # start from the begining, end at the goal stage
    s_start = map_node_info[starting_x][starting_y]
    print("Starting node is : " + str(starting_x) + ", " + str(starting_y))
    starting_node_string = ("Starting node is : " + str(starting_x) + ", " + str(starting_y))
    # detect the block
    detect(s_start, maze, map_node_info)
    s_goal = map_node_info[target_x][target_y]
    print("Ending node is : " + str(target_x) + ", " + str(target_y))
    path = point(-1, -1)
    while not (s_start.x == s_goal.x and s_start.y == s_goal.y):
        openlist = MinHeap()
        open_closed_list = [[False for i in range(maze_size)] for j in range(maze_size)]
        s_start.g = 0
        # push the start stage information to queue
        s_start.h = Manhattan(s_start, s_goal)
        MinHeap.push(openlist, s_start)

        # print("push point {} {}".format(s_start.x, s_start.y))

        ComputePath(maze, map_node_info, s_goal, openlist, open_closed_list
                    )
        track = traceback(map_node_info, s_goal)
        if len(openlist) == 0:
            print("I cannot reach the target.")
            return "NaN"

        s_start = take_action_Forward(track, maze, map_node_info, path)
        # print("move to point [{} {}]".format(s_start.x, s_start.y))
        # print("current path destination is [{} {}]".format(s_start.x, s_start.y))
    # print("goal point is [{} {}]".format(s_goal.x, s_goal.y))

    # final_track = final_trace(map_node_info, s_goal)
    ptr = path.next
    canvas.setFill('green')
    while (ptr.next != None):
        ptr = ptr.next

    while (ptr != None):
        current_x = ptr.x
        current_y = ptr.y
        canvas.drawOval(7 + current_x * 7 + 7 * .2, 7 + current_y * 7 + 7 * 0.2, 7 * 0.6, 7 * 0.6)
        ptr = ptr.parent

    destination = time.time()
    print("Time:", destination - origin)
    draw(maze, path)

    time1.valid_count += 1
    time1.average_time += (destination - origin)

    return (str(destination - origin))


def main_Backward():
    origin = time.time()
    # generate a random foggy map
    maze = setup()
    # generate a information map
    map_node_info = setup_node()


    counter = 0
    # origin from the begining, destination at the goal stage
    s_start = map_node_info[starting_x][starting_y]
    print("Starting node is : " + str(starting_x) + ", " + str(starting_y))
    # detect the block
    detect(s_start, maze, map_node_info)
    s_goal = map_node_info[target_x][target_y]
    print("Ending node is : " + str(target_x) + ", " + str(target_y))
    path = point(-1, -1)
    while not (s_start.x == s_goal.x and s_start.y == s_goal.y):
        openlist = MinHeap()
        open_closed_list = [[False for i in range(maze_size)] for j in range(maze_size)]
        s_goal.g = 0
        s_start.search = counter
        s_goal.search = counter
        # push the start stage information to queue
        s_goal.h = Manhattan(s_goal, s_start)
        MinHeap.push(openlist, s_goal)

        # print("push point {} {}".format(s_start.x, s_start.y))

        ComputePath(maze, map_node_info, s_start, openlist, open_closed_list)
        if len(openlist) == 0:
            print("I cannot reach the target.")
            return "NaN"


        s_start = take_action_Backward(s_start, maze, map_node_info, path)
        # print("move to point [{} {}]".format(s_start.x, s_start.y))
    # print("current path end is [{} {}]".format(path_ptr.x, path_ptr.y))
    # print("goal point is [{} {}]".format(s_goal.x, s_goal.y))

    # final_track = final_trace(map_node_info, s_goal)
    canvas.setFill("purple")
    ptr = path.next
    while(ptr.next != None):
        ptr = ptr.next

    while(ptr != None):
        current_x = ptr.x
        current_y = ptr.y
        canvas.drawOval(7 + current_x * 7 + 7* .2, 7 + current_y * 7 + 7*0.2, 7*0.6, 7*0.6)
        ptr = ptr.parent


    destination = time.time()
    print("Time:", destination - origin)
    draw(maze, path)

    time1.valid_count += 1
    time1.average_time += (destination - origin)
    return


def take_action_Backward(track, maze, map_node_info, path):
    x = track.x
    y = track.y
    # print("check position [{} {}]".format(x, y))
    position = None
    # keep moving until
    while (track != None):
        x = track.x
        y = track.y
        if (not map_node_info[x][y].isBlocked):
            detect(map_node_info[x][y], maze, map_node_info)
            position = track
            path.push(position.x, position.y)
            track = track.parent
        else:
            break
    # need to complete
    return position


if __name__ == "__main__":
    counter = 0
    maze = setup()
    win = GraphicsWindow(maze_size * 8 * 1.2, maze_size * 7 * 1.2)
    canvas = win.canvas()
    main_Forward()
    print("repeated forward expand node: {}".format(counter))
    win.wait()

    win = GraphicsWindow(maze_size * 8 * 1.2, maze_size * 7 * 1.2)
    canvas = win.canvas()
    main_Backward()
    print("repeated backward: {}".format(counter))
    win.wait()

import sys
from ezgraphics import GraphicsWindow
import numpy as np
from ClassForRepeated import *
import time
import random

maze_size = 101

starting_x = random.randint(0, 100)
starting_y = random.randint(0, 100)

target_x = random.randint(0, 100)
target_y = random.randint(0, 100)

# initialize the information matrix
def setup_node():
    # starting node

    grid = makeGrid()
    grid[starting_x][starting_y] = node()
    origin = grid[starting_x][starting_y]
    origin.x = starting_x
    origin.y = starting_y

    origin.h = np.abs(target_x-starting_x) + np.abs(target_y-starting_y)
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


'''
update the surrounding node of the current node s
It takes 3 parameters: 
Mazeinfor: the information matrix
current_node: the current stage node
counter: the iteration time of the A* search
Notice:
currently I don't think we need to check counter, because I think there
is no conditon in which g(succ(s, a) would be not bigger than g(s) + c(s, a)
Please tell me if I am wrong, because I am probably wrong
'''

'''
Important!!!!
declaration about why don't need to set g value to infinity:
because the cost always has a constant 1.
The close list and the open list can be merge into one list: open_closed_list,
because any node's g value that already in the open list don't need to be further updated
if location (x, y) in open_closed_list has value true, it is either in the close list,
or already in the open list which won't need to be modified.
'''


def update_neighbour_node(Maze, Mazeinfor, current_node, s_goal, Queue, open_closed_list):
    xcoor = current_node.x
    ycoor = current_node.y
    global counter
    # update right successor

    if (isValid(xcoor + 1, ycoor) and (not open_closed_list[xcoor + 1][ycoor])):
        open_closed_list[xcoor + 1][ycoor] = True
        if (not isinstance(Mazeinfor[xcoor + 1][ycoor], node)):
            Mazeinfor[xcoor + 1][ycoor] = node()
        successor = Mazeinfor[xcoor + 1][ycoor]
        if (not successor.isBlocked):
            successor.parent = current_node
            successor.g = current_node.g + 1
            successor.x = xcoor + 1
            successor.y = ycoor
            successor.h = Manhattan(successor, s_goal)
            MinHeap.push(Queue, successor)
            counter += 1
    # print("push point {} {}".format(xcoor + 1, ycoor))

    # update left successor
    if (isValid(xcoor - 1, ycoor) and (not open_closed_list[xcoor - 1][ycoor])):
        open_closed_list[xcoor - 1][ycoor] = True
        # this is equal to check if succ(s, a) < counter
        if (not isinstance(Mazeinfor[xcoor - 1][ycoor], node)):
            Mazeinfor[xcoor - 1][ycoor] = node()
        successor = Mazeinfor[xcoor - 1][ycoor]
        if (not successor.isBlocked):
            successor.parent = current_node
            successor.g = current_node.g + 1
            successor.x = xcoor - 1
            successor.y = ycoor
            successor.h = Manhattan(successor, s_goal)
            MinHeap.push(Queue, successor)
            counter += 1

    # print("push point {} {}".format(xcoor - 1, ycoor))

    # update downward successor
    if (isValid(xcoor, ycoor - 1) and (not open_closed_list[xcoor][ycoor - 1])):
        open_closed_list[xcoor][ycoor - 1] = True
        if (not isinstance(Mazeinfor[xcoor][ycoor - 1], node)):
            Mazeinfor[xcoor][ycoor - 1] = node()
        successor = Mazeinfor[xcoor][ycoor - 1]
        if (not successor.isBlocked):
            successor.parent = current_node
            successor.g = current_node.g + 1
            successor.x = xcoor
            successor.y = ycoor - 1
            successor.h = Manhattan(successor, s_goal)
            MinHeap.push(Queue, successor)
            counter += 1
    # print("push point {} {}".format(xcoor, ycoor - 1))

    # update upward successor
    if (isValid(xcoor, ycoor + 1) and (not open_closed_list[xcoor][ycoor + 1])):
        open_closed_list[xcoor][ycoor + 1] = True
        if (not isinstance(Mazeinfor[xcoor][ycoor + 1], node)):
            Mazeinfor[xcoor][ycoor + 1] = node()
        successor = Mazeinfor[xcoor][ycoor + 1]
        if (not successor.isBlocked):
            successor.parent = current_node
            successor.g = current_node.g + 1
            successor.x = xcoor
            successor.y = ycoor + 1
            successor.h = Manhattan(successor, s_goal)
            MinHeap.push(Queue, successor)
            counter += 1
    # print("push point {} {}".format(xcoor, ycoor + 1))
    return


# Make the grid, having the top left and bottom right block set to unblocked and seen
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
            # grid[100][89] = Cell(100, 89, True, False)
            # grid[99][89] = Cell(99, 89, True, False)
            # grid[98][89] = Cell(98, 89, True, False)
            # grid[97][89] = Cell(97, 89, True, False)

    return grid


'''
	detect function mimic the sensor detection, to be more specific, 
	suppose the agent standing at the stage s, using the real word information
	maze to update the surrounding information to map_node_info 
'''


def detect(s, maze, Mazeinfor):
    xcoor = s.x
    ycoor = s.y
    # print("locate at [{} {}]".format(xcoor, ycoor))
    if (isValid(xcoor - 1, ycoor)):
        # this is equal to check if succ(s, a) < counter
        if (not isinstance(Mazeinfor[xcoor - 1][ycoor], node)):
            Mazeinfor[xcoor - 1][ycoor] = node()
        successor = Mazeinfor[xcoor - 1][ycoor]
        successor.isBlocked = maze[xcoor - 1][ycoor].ifBlocked

    if (isValid(xcoor + 1, ycoor)):
        if (not isinstance(Mazeinfor[xcoor + 1][ycoor], node)):
            Mazeinfor[xcoor + 1][ycoor] = node()
        successor = Mazeinfor[xcoor + 1][ycoor]
        successor.isBlocked = maze[xcoor + 1][ycoor].ifBlocked

    if (isValid(xcoor, ycoor - 1)):
        if (not isinstance(Mazeinfor[xcoor][ycoor - 1], node)):
            Mazeinfor[xcoor][ycoor - 1] = node()
        successor = Mazeinfor[xcoor][ycoor - 1]
        successor.isBlocked = maze[xcoor][ycoor - 1].ifBlocked

    if (isValid(xcoor, ycoor + 1)):
        if (not isinstance(Mazeinfor[xcoor][ycoor + 1], node)):
            Mazeinfor[xcoor][ycoor + 1] = node()
        successor = Mazeinfor[xcoor][ycoor + 1]
        successor.isBlocked = maze[xcoor][ycoor + 1].ifBlocked

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


def draw(maze, path_list, off=10):
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
        xcoor = ptr.x
        ycoor = ptr.y
        canvas.setFill('red')
        canvas.drawOval(off + xcoor * cell_size + cell_size * .2, off + ycoor * cell_size + cell_size * .2,
                        cell_size * .6, cell_size * .6)
        # print("path at [{} {}]".format(xcoor, ycoor))
        ptr = ptr.parent

    win.wait()


def Manhattan(origin, goal):
    return (abs(goal.x - origin.x) + abs(goal.y - origin.y))


def ComputePath(Maze, Mazeinfor, s_goal, Queue, open_closed_list):
    # check whether queue is empty
    while (len(Queue) > 0):
        '''
        for i in range(len(Queue)):
            n = Queue._MinHeap__heap[i]
            print("[{} {} {} {}] ".format(n.x, n.y, n.g, n.g + n.h), destination = "")
        print(" ")
        print(" ")
        '''
        current_node = MinHeap.pop(Queue)
        # print("pop point {} {}".format(current_node.x, current_node.y))
        xcoor = current_node.x
        ycoor = current_node.y
        open_closed_list[xcoor][ycoor] = True
        if (current_node.x == s_goal.x and current_node.y == s_goal.y):
            return
        # update s's successors, executing step 5 to 13
        update_neighbour_node(Maze, Mazeinfor, current_node, s_goal, Queue, open_closed_list)


def traceback(map_node_info, s_goal):
    tracklist = node()
    ptr = s_goal
    # while ptr hasn't reach the origin node
    while (ptr.g != 0):
        tracklist.add_front(ptr)
        ptr = ptr.parentf

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
    while (not (ptr.x == starting_x and ptr.y == starting_y)):
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


def main():
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

        '''
        track record the current idea path from current start goal to the final goal
        '''
        ComputePath(maze, map_node_info, s_start, openlist, open_closed_list)
        if len(openlist) == 0:
            print("I cannot reach the target.")
            return

        '''
        while(ptr != None):
            print('track is [{} {}]'.format(ptr.x, ptr.y), end=' ')
            ptr = ptr.next
        '''
        s_start = take_action(s_start, maze, map_node_info, path)
        # print("move to point [{} {}]".format(s_start.x, s_start.y))
    # print("current path end is [{} {}]".format(path_ptr.x, path_ptr.y))
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
    draw(maze, path)

    return


if __name__ == "__main__":
    counter = 0
    main()
    print("repeated backward expand node: {}".format(counter))
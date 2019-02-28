import numpy as np
from ezgraphics import GraphicsWindow
import random

starting_x = random.randint(0, 100)
starting_y = random.randint(0, 100)

target_x = random.randint(0, 100)
target_y = random.randint(0, 100)


# Make the grid, having the top left and bottom right block set to unblocked and seen
def setup():
    grid = makeGrid()
    if (starting_x == target_x and starting_y == target_y):
        return False
    else:
        for i in range(101):
            for j in range(101):
                # Initialize each object
                if (i == starting_x and j == starting_y) or (i == target_x and j == target_y):
                    # Cell(x coor, y coor, if_blocked, if_visited)
                    grid[i][j] = Cell(i, j, False, True)
                else:
                    grid[i][j] = Cell(i, j, random_blocked())
    return grid


# Return false for unblocked, true for blocked
def random_blocked():
    check_blocked = np.random.choice([0, 1], 1, p=[0.3, 0.7])
    if check_blocked[0] == 1:
        return False
    return True


# making a grid as [101][101]
def makeGrid():
    grid = [[0 for x in range(101)] for y in range(101)]
    return grid


def draw(windowSize=1080, off=7):
    window = GraphicsWindow(windowSize, windowSize)
    canvas = window.canvas()
    offset_x = off  # Distance from left edge.
    offset_y = off  # Distance from top.
    cell_size = off  # Height and width of checkerboard squares.

    grid = setup()
    # start

    for i in range(101):  # Note that i ranges from 0 through 7, inclusive.
        for j in range(101):  # So does j.
            cell = grid[i][j]
            if not cell.ifBlocked:
                color = 'white'
            else:
                color = 'black'

            # if i == 0 and j == 0:
            #     color = 'red'
            canvas.setFill(color)
            # draw cell_size * cell_size rectangle at point (offset_x + i * cell_size, offset_y + j * cell_size)
            canvas.drawRect(offset_x + i * cell_size, offset_y + j * cell_size,
                            cell_size, cell_size)
    window.wait()


class Cell:
    def __init__(self, x_axis, y_axis, if_blocked, if_visited=False):
        self.x = x_axis
        self.y = y_axis
        self.ifBlocked = if_blocked
        self.ifVisited = if_visited

    def visit(self):
        self.ifVisited = True


draw()
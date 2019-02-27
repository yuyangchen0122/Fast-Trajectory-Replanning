class Cell:
    def __init__(self, x_axis, y_axis, if_blocked, if_visited=False):
        self.x = x_axis
        self.y = y_axis
        self.ifBlocked = if_blocked
        self.ifVisited = if_visited

    def visit(self):
        self.ifVisited = True


'''
this is the data type of each cell, which store the information of the location,
g value, h value, and indirectly store the f value, an boolean value determine whether
the cell is fully expanded or not
'''


class node:
    def __init__(self, block=False):
        self.x = 0
        self.y = 0
        self.h = 0
        self.g = 0
        self.search = 0
        self.parent = None
        self.next = None
        self.isBlocked = block

    def __lt__(self, other, c=100 * 100):
        # define a new rule to make comparison based on the f value of two cells
        return (self.g + self.h < other.g + other.h) or\
               (self.g + self.h == other.g + other.h and
                c * (self.g + self.h) - self.g < c * (other.g + other.h) - other.g)

    def add_front(self, node):
        list = self.next
        self.next = node
        node.next = list


class point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.next = None
        self.parent = None

    def push(self, x, y):
        new = point(x, y)
        list = self.next
        self.next = new
        new.next = list
        if (list != None):
            list.parent = new


class MinHeap:
    def __init__(self):
        self.heap = []
        self.last_index = -1

    def push(self, value):
        self.last_index += 1
        if self.last_index < len(self.heap):
            self.heap[self.last_index] = value
            self.siftup(self.last_index)
        else:
            self.heap.append(value)
            self.siftup(self.last_index)

    def pop(self):
        if self.last_index == -1:
            raise IndexError('pop from empty heap')

        min_value = self.heap[0]

        self.heap[0] = self.heap[self.last_index]
        self.last_index -= 1
        self.siftdown(0)

        return min_value

    def siftup(self, index, c=101 * 101):
        while index > 0:
            parent_index, parent_value = self.get_parent(index)
            if (parent_index == -1):
                break

            if parent_value < self.heap[index]:
                break

            self.heap[parent_index], self.heap[index] = \
                self.heap[index], self.heap[parent_index]

            index = parent_index

    def siftdown(self, index, c=101 * 101):
        while True:
            index_value = self.heap[index]

            left_child_index, left_child_value = self.get_left_child(index, index_value)
            right_child_index, right_child_value = self.get_right_child(index, index_value)
            if (left_child_index == None or right_child_index == None):
                break

            if index_value < left_child_value and index_value < right_child_value:
                break

            else:
                if left_child_value < right_child_value:
                    new_index = left_child_index
                else:
                    new_index = right_child_index

                self.heap[new_index], self.heap[index] = \
                    self.heap[index], self.heap[new_index]

                index = new_index

    def get_parent(self, index):
        if index == 0:
            return -1, None
        parent_index = (index - 1) // 2

        return parent_index, self.heap[parent_index]

    def get_left_child(self, index, default_value):
        left_child_index = 2 * index + 1

        if left_child_index > self.last_index:
            return None, default_value

        return left_child_index, self.heap[left_child_index]

    def get_right_child(self, index, default_value):
        right_child_index = index * 2 + 2

        if right_child_index > self.last_index:
            return None, default_value

        return right_child_index, self.heap[right_child_index]

    def __len__(self):
        return self.last_index + 1


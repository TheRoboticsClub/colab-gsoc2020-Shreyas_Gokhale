#!/usr/bin/env python2
# Ref: https://gist.github.com/Nicholas-Swift/003e1932ef2804bebef2710527008f44#file-astar-py


class Node:
    """
    Node class for A* path planner

    g = distance between current node and start node
    h = heuristic = distance form current node -> end node =  x^2 + y^2
    f = total cost =  h + g
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


class Astar:
    """ A* path planner"""

    def __init__(self, gridmap, start, end):

        self.closed_list = []
        self.open_list = []
        self.start_node = Node(None, start)
        self.end_node = Node(None, end)
        self.map = gridmap

        # Create start and end node
        self.start_node.g = self.start_node.h = self.start_node.f = 0
        self.end_node.g = self.end_node.h = self.end_node.f = 0

        # Add the start node
        self.open_list.append(self.start_node)

    def get_path(self):
        print("Getting Path")
        #
        # print("Start Node")
        # print(self.start_node.parent)
        # print(self.start_node.position)
        # print(self.start_node.g)
        # print(self.start_node.h)
        # print(self.start_node.f)
        #
        # print("End Node")
        # print(self.end_node.parent)
        # print(self.end_node.position)
        # print(self.end_node.g)
        # print(self.end_node.h)
        # print(self.end_node.f)

        # Loop until you find the end
        while len(self.open_list) > 0:
            # print(self.open_list)
            # print(self.closed_list)

            # Get the current node
            current_node = self.open_list[0]
            current_index = 0
            for index, item in enumerate(self.open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current off open list, add to closed list
            self.open_list.pop(current_index)
            self.closed_list.append(current_node)

            # Found the goal
            if current_node == self.end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]  # Return reversed path

            # Generate children
            children = []
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1),
                                 (1, 1)]:  # Adjacent squares

                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # Make sure within range
                if node_position[0] > (len(self.map) - 1) or node_position[0] < 0 or node_position[1] > (
                        len(self.map[len(self.map) - 1]) - 1) or node_position[1] < 0:
                    continue

                # Make sure walkable terrain
                if self.map[node_position[0]][node_position[1]] != 0:
                    continue

                # Create new node
                new_node = Node(current_node, node_position)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:

                # Child is on the closed list
                for closed_child in self.closed_list:
                    if child == closed_child:
                        continue

                # Create the f, g, and h values
                child.g = current_node.g + 1
                child.h = ((child.position[0] - self.end_node.position[0]) ** 2) + (
                        (child.position[1] - self.end_node.position[1]) ** 2)
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in self.open_list:
                    if child == open_node and child.g > open_node.g:
                        continue

                # Add the child to the open list
                self.open_list.append(child)

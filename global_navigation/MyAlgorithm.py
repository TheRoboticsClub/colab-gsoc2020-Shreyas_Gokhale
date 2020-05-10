from sensors import sensor
import numpy as np
import cv2

import threading
import time
import AStar
from datetime import datetime

time_cycle = 80


class MyAlgorithm(threading.Thread):

    def __init__(self, grid, sensor, vel):
        self.sensor = sensor
        self.grid = grid
        self.vel = vel
        sensor.getPathSig.connect(self.generatePath)

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

    def run(self):

        while not self.kill_event.is_set():
            start_time = datetime.now()
            if not self.stop_event.is_set():
                self.execute()

            finish_time = datetime.now()
            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            # print (ms)
            if ms < time_cycle:
                time.sleep((time_cycle - ms) / 1000.0)

    def stop(self):
        self.stop_event.set()

    def play(self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()

    def kill(self):
        self.kill_event.set()

    def drawPath(self):
        print("Drawing path")

    """
        Write in this method the code necessary for looking for the shorter
        path to the desired destiny.
        The destiny is chosen in the GUI, making double click in the map.
        This method will be call when you press the Generate Path button. 
        Call to grid.setPath(path) method for setting the path. 
    """

    def generatePath(self):
        print("LOOKING FOR SHORTER PATH")
        map_grid = self.grid.getMap()
        dest = self.grid.getDestiny()
        source = self.grid.getPose()

        # Note: The grid coordinates are inverted

        if map_grid[dest[1]][dest[0]] < 125:
            print("Clicked point not an empty space!")
            return 0
        else:
            print("Point selected has colour")
            map_copy = np.array(map_grid)
            dest_transpose = (dest[1], dest[0])
            source_transpose = (source[1], source[0])

            # Is value < 125 -> True ->  int -> 1
            map_copy = (map_copy < 125).astype(int)

            a = time.time()
            a_star = AStar.Astar(map_copy, source_transpose, dest_transpose)
            path = a_star.get_path()
            t = time.time() - a

            print("Shortest Path found in: %f Seconds" % t)

            self.grid.setPathFinded()
            for node in path:
                self.grid.setPathVal(node[1], node[0], 1)
        pass

    """
        Write in this method the code necessary for going to the desired place,
        once you have generated the shorter path.
        This method will be periodically called after you press the GO! button.
    """

    def execute(self):
        # Add your code here
        print("GOING TO DESTINATION")
        pass
        # EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        # self.vel.setV(10)
        # self.vel.setW(5)
        # self.vel.sendVelocities()

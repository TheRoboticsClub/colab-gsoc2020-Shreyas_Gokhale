from sensors import sensor
import numpy as np
import cv2

import threading
import time
import AStar
from datetime import datetime
import math

time_cycle = 80

# Motion Planning:
# https://github.com/AtsushiSakai/PythonRobotics/blob/3607d72b60cd500806e0f026ac8beb82850a01f9/PathTracking/move_to_pose/move_to_pose.py#L127

Kp_rho = 4
Kp_alpha = 6
Kp_beta = -3

def getDiff(current_pose, goal):
    distance = math.hypot(goal[0] - current_pose[0], goal[1] - current_pose[1])
    angle = math.atan2(goal[0] - current_pose[0], goal[1] - current_pose[1])
    return distance, angle


class MyAlgorithm(threading.Thread):

    def __init__(self, grid, sensor, vel):
        self.path = None
        self.goal = None
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
            self.path = path[1::3]
            self.goal = path[-1]
        pass

    """
        Write in this method the code necessary for going to the desired place,
        once you have generated the shorter path.
        This method will be periodically called after you press the GO! button.
    """

    def execute(self):
        # Add your code here
        print("GOING TO DESTINATION")
        print self.path
        print("starting")
        reached_destination = False
        destination = self.grid.getDestiny()
        goal = self.path[-1]


        try:
            old_goal = self.goal
        except (ValueError, TypeError):
            self.goal = goal
            old_goal = goal


        current_pose = self.grid.getPose()
        current_theta = self.sensor.getRobotTheta()

        if current_pose == old_goal: #abs(goal[0]**2 - old_goal[0]**2) < 10:
            self.goal = self.path.pop()

        print goal
        print old_goal
        print current_pose

        x = current_pose[0]
        y = current_pose[1]
        theta = current_theta

        x_diff = self.goal[0] - x
        y_diff = self.goal[1] - y

        goal_dist, goal_theta = getDiff(current_pose, goal)



        # Restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff)
                 - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (goal_theta - theta - alpha + np.pi) % (2 * np.pi) - np.pi

        v = Kp_rho * rho
        w = Kp_alpha * alpha + Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        print("Resultant W: %f " % w)
        self.vel.setW(w)
        print("Resultant V: %f"%v)
        self.vel.setV(v)

        pass

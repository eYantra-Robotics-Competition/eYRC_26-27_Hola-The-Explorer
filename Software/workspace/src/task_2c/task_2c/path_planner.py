#!/usr/bin/env python3
# Copyright (c) 2026 e-Yantra, IIT Bombay. All rights reserved.
# These simulation files and source code are the intellectual property of e-Yantra,
# IIT Bombay, provided solely for eYRC 2026-27 (Theme: Hola The Explorer).
# Sharing or redistribution of this material, in whole or in part, is not permitted.

'''
*****************************************************************************************
*
*        =============================================
*           Hola The Explorer (HE) Theme (eYRC 2026-27)
*        =============================================
*
*  This script is to implement Task 2C of Hola The Explorer (HE) Theme (eYRC 2026-27).
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:      [ Names of team members who worked on this file, separated by comma ]
# Filename:         path_planner.py
# Functions:        [ Add every extra helper function you write to this list ]
# Global variables: [ Add every extra global variable you declare to this list ]


############################ WHAT YOU HAVE TO DO ##############################
#
#  r1, r2 and r3 must each reach their goal without touching an obstacle or
#  each other. A wall with ONE gap crosses the arena and the goals cross
#  sides, so the robots must take the gap one at a time. This node plans all
#  three ONCE and publishes each on /rN/plan; path_follower.py drives them.
#
#    camera frame --frame_to_occupancy()--> grid       (given)
#                 --remove_drivable()-----> grid       (from Task 2B)
#                 --planning_grid()-------> blocked    (given)
#                 --plan_all()------------> plans      (YOURS)
#                 --to_path_msg()---------> /rN/plan   (from Task 2B)
#
#  A plan is [(t, x, y), ...]: be at (x, y) t seconds after setting off. All
#  three share one start time, so the TIMES are what keep the robots apart.
#
#  Frame (/rN/odom): origin at the arena's TOP-LEFT corner, x right, y DOWN.
#
#  Run:  ros2 launch hb_description task2c.launch.py
#        ros2 run task_2c path_planner
#        ros2 run task_2c path_follower
#
###############################################################################

import math

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

################# ADD EXTRA IMPORTS / GLOBALS HERE ############

###############################################################


# ----------------------------------------------------- arena (given, do not edit)
ROBOTS = ("r1", "r2", "r3")          # also the planning order
START_CIRCLES = [(0.8969, 2.2616), (1.2192, 2.2616), (1.5415, 2.2616)]
GOALS = {"r1": (1.9992, 0.7992),     # the goals cross sides
         "r2": (1.2192, 0.5392),
         "r3": (0.4392, 0.7992)}
ARENA_SIZE = 2.4384                  # m
ROBOT_RADIUS = 0.084                 # m
SAFETY_MARGIN = 0.030                # m, kept clear on top of the robot radius

# ----------------------------------------------------- camera (given)
CAMERA_URL = "http://127.0.0.1:8080/stream"
ARENA_LEFT, ARENA_TOP, ARENA_PIXELS = 304, 24, 671       # floor square in the image
GRID_CELLS = 244                     # occupancy grid is GRID_CELLS x GRID_CELLS
CELL_SIZE = ARENA_SIZE / GRID_CELLS  # m, about 1 cm
SAND_LOW, SAND_HIGH = (10, 40, 170), (30, 120, 255)      # sand colour, HSV

# ----------------------------------------------------- planning (tunable)
PLAN_CELL = 0.04                     # m, planning grid cell
PLAN_SPEED = 0.24                    # m/s, the speed the paths are timed at
ROBOT_CLEARANCE = 0.26               # m, keep two robots' plans this far apart
START_DELAY = 2.0                    # s, from publishing to setting off

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


######################### OCCUPANCY GRID ######################

def read_frame(url=CAMERA_URL):
    """Given. One BGR frame from the overhead camera."""
    cap = cv2.VideoCapture(url)
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError("no frame from %s -- is task2c.launch.py running?" % url)
    return frame


def crop_arena(frame):
    """Given. The arena floor only, ARENA_PIXELS square."""
    return frame[ARENA_TOP:ARENA_TOP + ARENA_PIXELS, ARENA_LEFT:ARENA_LEFT + ARENA_PIXELS]


def frame_to_occupancy(frame):
    """Given. Frame -> grid[row, col] = grid[y, x], True = obstacle.
    Only sand is free. To view: cv2.imshow("g", np.where(grid, 0, 255).astype(np.uint8))"""
    hsv = cv2.cvtColor(cv2.medianBlur(crop_arena(frame), 5), cv2.COLOR_BGR2HSV)
    obstacles = cv2.bitwise_not(cv2.inRange(hsv, SAND_LOW, SAND_HIGH))
    # drop specks and thin painted lines, then fill the holes inside each obstacle
    obstacles = cv2.morphologyEx(obstacles, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(obstacles, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(obstacles, contours, -1, 255, cv2.FILLED)
    # one pixel per cell; a cell is an obstacle if any part of it is
    small = cv2.resize(obstacles, (GRID_CELLS, GRID_CELLS), interpolation=cv2.INTER_AREA)
    return small > 0


def clear_circle(occupied, xy, radius):
    """Given. Free every cell within `radius` m of point xy (in place)."""
    rows, cols = np.indices(occupied.shape)
    x, y = (cols + 0.5) * CELL_SIZE, (rows + 0.5) * CELL_SIZE
    occupied[(x - xy[0]) ** 2 + (y - xy[1]) ** 2 <= radius ** 2] = False


def remove_drivable(frame, occupied, robots, goals):
    """Free what a robot may drive over; return the grid.

    frame_to_occupancy() marks everything that is not sand as an obstacle --
    including the painted floor, the robots and the goal dots, which leaves
    NO path. Keep the wall and pillars (and each pad's payload)."""
    ##############  ADD YOUR CODE HERE  ##############
    # TODO: from Task 2B. Check it here: the gap, every start and every goal
    # must come out free.
    return occupied
    ##################################################


######################### PLANNING GRID #######################

def planning_grid(occupied):
    """Given. Grid -> blocked[i, j] = blocked[x, y] (NOTE: x first) of PLAN_CELL
    cells. Obstacles grow by ROBOT_RADIUS + SAFETY_MARGIN, so plan the robot's
    centre as a point. The arena's edge is a wall."""
    pad = ROBOT_RADIUS + SAFETY_MARGIN
    k = int(math.ceil(pad / CELL_SIZE))
    disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1))
    inflated = cv2.dilate(occupied.astype(np.uint8), disc) > 0

    # a planning cell is blocked if the robot's centre, standing on the cell's
    # centre, would be too close to an obstacle
    n = int(round(ARENA_SIZE / PLAN_CELL))
    centre = (np.arange(n) + 0.5) * PLAN_CELL
    fine = np.minimum((centre / CELL_SIZE).astype(int), GRID_CELLS - 1)
    blocked = inflated[np.ix_(fine, fine)].T        # [row=y, col=x] -> [i=x, j=y]
    near_wall = (centre < pad) | (centre > ARENA_SIZE - pad)
    blocked[near_wall, :] = True
    blocked[:, near_wall] = True
    return blocked


def to_cell(p):
    """Given. Point (x, y) in m -> planning cell (i, j)."""
    n = int(round(ARENA_SIZE / PLAN_CELL))
    return (min(max(int(p[0] / PLAN_CELL), 0), n - 1), min(max(int(p[1] / PLAN_CELL), 0), n - 1))


def to_world(c):
    """Given. Planning cell (i, j) -> its centre (x, y) in m."""
    return ((c[0] + 0.5) * PLAN_CELL, (c[1] + 0.5) * PLAN_CELL)


def nearest_free(blocked, p, max_r=0.30):
    """Given. The free cell closest to point p, or None within max_r m."""
    c = to_cell(p)
    if not blocked[c]:
        return c
    free = [(i, j) for i in range(blocked.shape[0]) for j in range(blocked.shape[1])
            if not blocked[i, j] and abs(i - c[0]) * PLAN_CELL <= max_r
            and abs(j - c[1]) * PLAN_CELL <= max_r]
    return min(free, key=lambda q: math.dist(to_world(q), p)) if free else None


def plan_all(blocked, starts):
    """starts {robot: (x, y)} -> {robot: [(t, x, y), ...]}, every robot from
    (0.0, *starts[robot]) to exactly GOALS[robot], with no collisions; or None."""
    ##############  ADD YOUR CODE HERE  ##############
    # TODO (prioritised planning is a good start):
    #   1. search over (cell, time step); a robot may move OR wait. One step
    #      takes PLAN_CELL / PLAN_SPEED seconds
    #   2. plan robots in ROBOTS order. After each, reserve the cells within
    #      ROBOT_CLEARANCE of it at every step; later robots avoid them
    #   3. no head-on swaps, and a robot parked on its goal stays reserved
    pass
    ##################################################


######################### NODE ################################

class PathPlanner(Node):
    """Given, except to_path_msg(). Waits for every /rN/odom, plans once, and
    publishes each path on /rN/plan (latched) with one shared start time."""

    def __init__(self):
        super().__init__("task_2c_path_planner")
        self.pose, self.pubs = {}, {}
        for r in ROBOTS:
            self.create_subscription(Odometry, f"/{r}/odom", self._odom_cb(r), 10)
            self.pubs[r] = self.create_publisher(Path, f"/{r}/plan", LATCHED)
        self.planned = False
        self.create_timer(0.2, self.tick)
        self.get_logger().info("task 2C planner: goals %s" % GOALS)

    def _odom_cb(self, robot):
        def cb(msg):
            p = msg.pose.pose.position
            self.pose[robot] = (p.x, p.y)
        return cb

    def to_path_msg(self, plan, start):
        """[(t, x, y), ...] and start (rclpy Time) -> nav_msgs/Path."""
        ##############  ADD YOUR CODE HERE  ##############
        # TODO: from Task 2B
        pass
        ##################################################

    def tick(self):
        if self.planned or len(self.pose) < len(ROBOTS):
            return                              # wait for every robot's odom
        self.planned = True

        frame = read_frame()
        occupied = remove_drivable(frame, frame_to_occupancy(frame),
                                   robots=list(self.pose.values()), goals=list(GOALS.values()))
        blocked = planning_grid(occupied)
        self.get_logger().info("occupancy grid: %.1f%% obstacle; planning grid %d x %d"
                               % (100.0 * occupied.mean(), *blocked.shape))

        plans = plan_all(blocked, self.pose)
        if plans is None:
            self.get_logger().error("no conflict-free joint plan. Starts and goals, as the planner saw them:")
            for r in ROBOTS:
                self.get_logger().error("  %s start %s %s -> goal %s %s" % (
                    r, self.pose[r], "BLOCKED" if blocked[to_cell(self.pose[r])] else "free",
                    GOALS[r], "BLOCKED" if blocked[to_cell(GOALS[r])] else "free"))
            return

        start = self.get_clock().now() + Duration(seconds=START_DELAY)   # shared by all three
        for r in ROBOTS:
            self.pubs[r].publish(self.to_path_msg(plans[r], start))
            length = sum(math.dist(a[1:], b[1:]) for a, b in zip(plans[r], plans[r][1:]))
            self.get_logger().info("%s: published /%s/plan, %d poses, %.2f m, arrives at t=%.1f s"
                                   % (r, r, len(plans[r]), length, plans[r][-1][0]))


def main():
    rclpy.init()
    node = PathPlanner()
    try:
        rclpy.spin(node)                        # keeps the latched paths available
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

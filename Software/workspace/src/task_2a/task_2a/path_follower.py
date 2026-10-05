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
*  This script is to implement Task 2A of Hola The Explorer (HE) Theme (eYRC 2026-27).
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:      [ Names of team members who worked on this file, separated by comma ]
# Filename:         path_follower.py
# Functions:        [ Add every extra helper function you write to this list ]
# Global variables: [ Add every extra global variable you declare to this list ]


############################ WHAT YOU HAVE TO DO ##############################
#
#  Drive r1, r2 and r3 AT THE SAME TIME, each in a straight line to its goal
#  in GOALS, holding its starting heading, and park it there.
#
#   1. Paste in your Task 1B / 1C code   (body_velocity_to_wheel_speeds, ...)
#   2. Write wrap(), to_body() and the GoToPoint PID controller
#   3. Complete RobotNode                (main() runs one per robot)
#
#  Frame (/rN/odom): origin at the arena's TOP-LEFT corner, x right, y DOWN,
#  metres. Yaw 0 along +x, growing clockwise; the robots start at -pi/2.
#
#  Run:  ros2 launch hb_description task2a.launch.py
#        ros2 run task_2a path_follower
#
###############################################################################

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

################# ADD EXTRA IMPORTS / GLOBALS HERE ############

###############################################################


# ----------------------------------------------------- arena (given, do not edit)
ARENA_SIZE = 2.4384           # m, the floor is square
ROBOT_RADIUS = 0.084          # m

START = {"r1": (0.8969, 2.2616),
         "r2": (1.2192, 2.2616),
         "r3": (1.5415, 2.2616)}
GOALS = {"r1": (0.6692, 0.8692),
         "r2": (1.2192, 0.6692),
         "r3": (1.7692, 0.8692)}
ORDER = ("r1", "r2", "r3")    # one node per robot

# ----------------------------------------------------- yours to set
CONTROL_HZ = 0.0              # how often tick() runs
GOAL_TOLERANCE = 0.0          # m, how close counts as "at the goal"
SETTLE_TICKS = 0              # ticks in a row inside GOAL_TOLERANCE = arrived

# ----------------------------------------------------- robot (same as Task 1B)
WHEEL_RADIUS_M = 0.0255       # m
CHASSIS_RADIUS_M = 0.06412    # m, chassis centre to each wheel's axle
WHEEL_ANGLES_RAD = np.radians([0.0, 0.0, 0.0])   # TODO: from Task 1B, [left, right, back]
IK_MATRIX = np.zeros((3, 3))                     # TODO: from Task 1B
_CTRL_LIMIT = 30.0            # rad/s, the wheels' ctrlrange in the robot's MJCF:
                              # faster commands are clamped by the simulation


##################### FROM TASK 1B / 1C #######################

def body_velocity_to_wheel_speeds(vx, vy, w):
    """Body twist (vx, vy, w) -> wheel speeds [left, right, back], rad/s."""
    ##############  ADD YOUR CODE HERE  ##############
    # TODO: Task 1B
    pass
    ##################################################


def body_to_wheels(vx, vy, wz):
    """Like body_velocity_to_wheel_speeds(), but kept inside +/-_CTRL_LIMIT.
    Returns a list."""
    ##############  ADD YOUR CODE HERE  ##############
    # TODO: Task 1C. Scale all three wheels down together, do not clip each
    # one: clipping changes the direction the robot drives in.
    pass
    ##################################################


def yaw_from_quat(w, x, y, z):
    """Quaternion -> yaw, radians."""
    ##############  ADD YOUR CODE HERE  ##############
    # TODO: Task 1C
    pass
    ##################################################


######################### NEW IN 2A ###########################

def wrap(a):
    """Angle -> the same angle in (-pi, pi]. 350 degrees becomes -10."""
    ##############  ADD YOUR CODE HERE  ##############
    # TODO
    pass
    ##################################################


def to_body(vx_a, vy_a, yaw):
    """Arena-frame velocity -> body-frame velocity (vx, vy).
    Check: at yaw = -pi/2 (facing up), arena (0, -v) must give body (v, 0)."""
    ##############  ADD YOUR CODE HERE  ##############
    # TODO
    pass
    ##################################################


def frame_check(x, y, size=ARENA_SIZE):
    """Given. A complaint if (x, y) is not in the /odom frame, else None."""
    if not (0.0 <= x <= size and 0.0 <= y <= size):
        return ("odom pose (%.3f, %.3f) is outside 0..%.4f: this code works "
                "in the top-left-origin arena frame, not MuJoCo's "
                "centre-origin one" % (x, y, size))
    return None


class GoToPoint:
    """PID controller: drive one robot straight at a point, holding a fixed yaw.
    One instance per robot -- the PID terms keep state between steps."""

    def __init__(self, v_max=0.0, w_max=0.0, kp=0.0, ki=0.0, kd=0.0,
                 kyaw=0.0, kiyaw=0.0, kdyaw=0.0, tol=0.0, dt=0.0):
        self.v_max, self.w_max = v_max, w_max            # m/s, rad/s caps
        self.kp, self.ki, self.kd = kp, ki, kd           # position PID
        self.kyaw, self.kiyaw, self.kdyaw = kyaw, kiyaw, kdyaw   # yaw PID
        self.tol = tol                                   # m, stop inside this
        self.dt = dt                                     # s, 1 / CONTROL_HZ
        ##############  ADD YOUR CODE HERE  ##############
        # TODO: PID state -- integrals and previous errors for (x, y) and yaw
        ##################################################

    def step(self, pose, target, hold_yaw):
        """pose (x, y, yaw), target (x, y), hold_yaw -> (wheels, distance)."""
        ##############  ADD YOUR CODE HERE  ##############
        # TODO:
        #   1. error and distance from pose to target, in the arena frame
        #   2. arena velocity from a PID on that error: 0 inside tol, size
        #      capped at v_max, integral limited so it cannot wind up
        #   3. wz from a PID on wrap(hold_yaw - yaw), capped at w_max
        #   4. to_body(), then body_to_wheels()
        pass
        ##################################################


######################### ONE NODE PER ROBOT ##################

class RobotNode(Node):
    """Drives one robot to GOALS[robot]. Nodes: task_2a_r1, _r2, _r3.
    No robot moves until all three are in `ready`, so they start together."""

    def __init__(self, robot, ready, on_arrival):
        super().__init__(f"task_2a_{robot}")
        self.robot = robot
        self.ready = ready            # shared by the three nodes
        self.on_arrival = on_arrival  # call once: on_arrival(robot, seconds)

        self.pose = None              # (x, y, yaw) from /<robot>/odom
        self.hold_yaw = None          # yaw on the first odom message
        self.settled = 0
        self.arrived = False
        self.t_start = None
        ##############  ADD YOUR CODE HERE  ##############
        # TODO:
        #   - subscribe to /<robot>/odom (self.odom_cb); publisher for
        #     /<robot>/wheel_commands in self.pub
        #   - self.ctl = a GoToPoint for this robot
        #   - a timer calling self.tick at CONTROL_HZ
        ##################################################

    def odom_cb(self, msg):
        ##############  ADD YOUR CODE HERE  ##############
        # TODO: self.pose = (x, y, yaw). First message only: set
        # self.hold_yaw, log frame_check(x, y) if it complains, and add
        # self.robot to self.ready.
        pass
        ##################################################

    def send(self, wheels):
        self.pub.publish(Float64MultiArray(data=[float(w) for w in wheels]))

    def tick(self):
        if len(self.ready) < len(ORDER):
            return                                   # wait for every robot's odom
        if self.t_start is None:
            self.t_start = self.get_clock().now()
        ##############  ADD YOUR CODE HERE  ##############
        # TODO:
        #   - arrived: send zeros and return
        #   - else: wheels from self.ctl.step(self.pose, GOALS[self.robot],
        #     self.hold_yaw), and send them
        #   - after SETTLE_TICKS in a row within GOAL_TOLERANCE: set
        #     self.arrived, log "<robot> reached its goal (... mm out)", and
        #     call self.on_arrival(self.robot, seconds since self.t_start)
        pass
        ##################################################


def main():
    """Given. One RobotNode per robot; logs "task 2A complete" at the end."""
    rclpy.init()
    ready = set()
    arrived = {}

    def on_arrival(robot, t):
        arrived[robot] = t
        if len(arrived) == len(ORDER):
            nodes[0].get_logger().info("task 2A complete in %.1fs" % max(arrived.values()))

    nodes = [RobotNode(r, ready, on_arrival) for r in ORDER]
    executor = MultiThreadedExecutor(num_threads=len(ORDER))
    for n in nodes:
        executor.add_node(n)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for n in nodes:
            try:
                n.send((0.0, 0.0, 0.0))          # leave nothing driving
            except Exception:                    # no publisher yet, or Ctrl+C took the context down
                pass
            n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

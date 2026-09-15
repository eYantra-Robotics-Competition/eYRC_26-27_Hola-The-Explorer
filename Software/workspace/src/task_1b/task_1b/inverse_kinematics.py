# Copyright (c) 2026 e-Yantra, IIT Bombay. All rights reserved.
# These simulation files and source code are the intellectual property of e-Yantra,
# IIT Bombay, provided solely for eYRC 2026-27 (Theme: Hola The Explorer).
# Sharing or redistribution of this material, in whole or in part, is not permitted.

'''
*****************************************************************************************
*
*        =============================================
*           Hola The Explorer (HE) Theme (eYRC 2025-26)
*        =============================================
*
*  This script is to implement Task 1B of Hola The Explorer (HE) Theme (eYRC 2025-26).
*
*****************************************************************************************
'''

#!/usr/bin/env python3
"""
This node subscribes to /cmd_vel (geometry_msgs/Twist: linear.x=vx,
linear.y=vy, angular.z=w) and is supposed to publish the three wheel
speeds it implies on /wheel_commands (std_msgs/Float64MultiArray, ordered
[left, right, back]) for hb_description's bridge_node to apply.

Everything below is scaffolding -- the publisher, the subscriber, the node
setup -- except for body_velocity_to_wheel_speeds(), which is the one part
you need to write. Leave the rest of the file alone.

Usage (once body_velocity_to_wheel_speeds is implemented):
    ros2 launch hb_description task1b.launch.py   # in one terminal
    ros2 run task_1b inverse_kinematics             # in another
    ros2 run task_1b cmd_vel_publisher               # in a third
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# ---------------------------------------------------------------------------
# Given: the robot's physical dimensions. You don't need to re-derive these --
# they describe the actual hardware/simulated robot -- but you will need them
# inside your inverse kinematics.
# ---------------------------------------------------------------------------

# Radius of each wheel, in metres.
WHEEL_RADIUS_M = 0.0255

# Distance from the chassis's centre to each wheel's axle, in metres.
CHASSIS_RADIUS_M = 0.06412

# The direction each wheel DRIVES in (not where its axle points -- the axle is
# perpendicular to this, pointing radially outward). Angles are in radians,
# measured counter-clockwise from the robot's own +x axis, in the ROS body
# frame: +x forward, +y left, +w counter-clockwise.
# Order matches /wheel_commands: [left, right, back].
WHEEL_ANGLES_RAD = np.radians([30.0, 150.0, 270.0])


def body_velocity_to_wheel_speeds(vx, vy, w):   
    """Convert a desired body velocity into the three wheel speeds.

    Args:
        vx: forward/backward velocity of the chassis, in m/s.
        vy: sideways velocity of the chassis, in m/s.
        w:  yaw rate of the chassis, in rad/s (positive = CCW).

    Returns:
        A sequence of exactly 3 numbers -- the angular speed each wheel
        needs to spin at, in rad/s -- ordered [left, right, back] to match
        /wheel_commands.

    TODO: derive and implement the inverse kinematics matrix for this
    robot's 3-wheel omni (kiwi) drive. For each wheel i, driving along
    WHEEL_ANGLES_RAD[i] and sitting CHASSIS_RADIUS_M from the centre:

      1. Each wheel's axle points radially outward, perpendicular to the
         direction it drives in; the wheel can only drive (no slip) along
         WHEEL_ANGLES_RAD[i] -- the passive rollers absorb any motion
         along the axle itself.
      2. Write the chassis velocity at that wheel's position as a function
         of (vx, vy, w).
      3. Project that velocity onto the wheel's driven direction, and
         relate it to the wheel's angular speed through WHEEL_RADIUS_M.

    That gives one equation per wheel; stack the three into a matrix and
    you have your inverse kinematics.
    """
    # ----- YOUR CODE HERE -----------------------------------------------
    raise NotImplementedError("TODO: implement body_velocity_to_wheel_speeds")
    # ----------------------------------------------------------------------


class InverseKinematicsNode(Node):

    def __init__(self):
        super().__init__("inverse_kinematics_node")
        self._pub = self.create_publisher(Float64MultiArray, "/wheel_commands", 10)
        self._sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.get_logger().info("Listening on /cmd_vel, publishing to /wheel_commands.")

    def _on_cmd_vel(self, msg: Twist):
        wheel_speeds = body_velocity_to_wheel_speeds(msg.linear.x, msg.linear.y, msg.angular.z)
        out = Float64MultiArray()
        out.data = list(wheel_speeds)
        self._pub.publish(out)


def main():
    rclpy.init()
    node = InverseKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

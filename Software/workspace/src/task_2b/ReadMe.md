# Task 2B - Mind the Obstacles

## What to do

Drive one robot, `r2`, to three goals in order, **without touching an obstacle**. Standing and toppled pillars block the way, so a straight line to a goal hits them: plan a path around them.

The work is split across two nodes:

| Node | Does |
| --- | --- |
| `path_planner` | builds an occupancy grid from one frame of the overhead camera, plans a timed path through the three goals, and publishes it on `/r2/plan` (latched) |
| `path_follower` | receives the path and drives the robot along it, stopping on each goal |

Two rules for `/r2/plan` (`nav_msgs/Path`):

- **Put the time in the stamps.** The path's `header.stamp` is the moment the robot sets off. Each pose's `header.stamp` is that plus the time the robot should be there. To make the robot wait on a goal, add the goal's pose again with a later time.
- **It is latched** (`TRANSIENT_LOCAL`), so the follower gets the path even if it starts after the planner.

The occupancy grid frees only sand-coloured floor, so the robot, the goal dots, the start circles and the painted floor come out as obstacles too, and the raw grid has **no path** to any goal. Freeing those is part of your job; the pillars must stay obstacles.

It's correct when the follower logs `r2 reached goal k of 3 (… mm out)` for each goal, then `task 2B complete`, and in `/r2/odom` the robot's centre never comes within 84 mm (its radius) of an obstacle.

## Files to edit

```
Software/workspace/src/task_2b/task_2b/path_planner.py
Software/workspace/src/task_2b/task_2b/path_follower.py
```

Fill in the `ADD YOUR CODE HERE` blocks, in this order:

1. `path_planner.py`: `remove_drivable()`, `plan_route()` and `to_path_msg()`. Any planning method works; A* with the zig-zags straightened out is a good start.
2. `path_follower.py`: paste your Task 2A kinematics and `GoToPoint` into the **FROM TASK 2A** section, with your tuned gains as `GoToPoint`'s defaults, then write `path_to_traj()`.

Get the robot to goal 1 before you plan all three. Leave the camera, grid and node code as given.

## Run

```bash
cd ~/eYRC_26-27_Hola-The-Explorer
git pull
cd Software/workspace
colcon build
source install/setup.bash
```

```bash
# Terminal 1 - simulation
ros2 launch hb_description task2b.launch.py

# Terminal 2 - planner
ros2 run task_2b path_planner

# Terminal 3 - follower
ros2 run task_2b path_follower
```

The planner logs `published /r2/plan`. If it logs `no route`, it also prints which start or goal is `BLOCKED` on your grid: fix `remove_drivable()` and run it again. Check the path by hand:

```bash
ros2 topic echo /r2/plan --qos-durability transient_local --once
```

# Task 2C - Move as a Team

## What to do

Run all three robots, `r1`, `r2` and `r3`, **at once**, and drive each one to its own goal without touching an obstacle **or another robot**.

A wall of pillars with **one gap** runs across the arena, and the goals cross sides: the left robot ends on the right, and the right robot on the left. Plan each robot on its own, as in Task 2B, and all three reach the gap together and collide. Your planner decides the order in which they pass through the gap, and makes the others wait.

| Node | Does |
| --- | --- |
| `path_planner` | builds the occupancy grid, plans all three robots in space **and** time, and publishes each path on `/rN/plan` (latched) |
| `path_follower` | runs one node per robot, `task_2c_r1` … `task_2c_r3`; each drives its robot along its own path, all three at once |

All three paths share **one** `header.stamp`, the moment the robots set off, and each pose's stamp is that plus the time the robot should be there. The followers never talk to each other: they stay apart because each keeps its robot on the same clock, so if your planned times keep the robots apart, so will the robots.

It's correct when every robot ends on its goal, the follower logs `<robot> reached its goal (… mm out)` for each, then `task 2C complete`, and in `/rN/odom` no two robots come within 168 mm (one chassis width) of each other.

## Files to edit

```
Software/workspace/src/task_2c/task_2c/path_planner.py
Software/workspace/src/task_2c/task_2c/path_follower.py
```

1. Copy in from your Task 2B files: `remove_drivable()` and `to_path_msg()` in `path_planner.py`, and `path_to_traj()` plus the **FROM TASK 2A** section in `path_follower.py`. Check `remove_drivable()` on this arena: the gap, every start and every goal must come out free.
2. Write `plan_all()`. Start with **prioritised planning**: plan `r1` as in Task 2B and record the cells it uses at every time step (a reservation table), plan `r2` around them, then `r3` around both. A robot may wait in a cell to let another one pass.

Plan one robot with your Task 2B planner before planning three. Leave the camera, grid and node code as given.

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
ros2 launch hb_description task2c.launch.py

# Terminal 2 - planner
ros2 run task_2c path_planner

# Terminal 3 - follower
ros2 run task_2c path_follower
```

The planner logs `published /rN/plan` for each robot. If it logs `no conflict-free joint plan`, check your grid first: the gap, every start and every goal must be free. Check a path by hand:

```bash
ros2 topic echo /r1/plan --qos-durability transient_local --once
```

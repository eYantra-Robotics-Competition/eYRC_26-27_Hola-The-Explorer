# Task 2A - Three Robots, Three Goals

## What to do

Drive three robots, `r1`, `r2` and `r3`, to three fixed goals **at the same time**, from one file. The straight line from each start circle to its goal is clear, and the three lines never cross, so no robot has to avoid anything.

Each robot has its own topics, under its name:

| Robot | Starts on | Reads | Writes |
| --- | --- | --- | --- |
| `r1` | left start circle | `/r1/odom` | `/r1/wheel_commands` |
| `r2` | middle start circle | `/r2/odom` | `/r2/wheel_commands` |
| `r3` | right start circle | `/r3/odom` | `/r3/wheel_commands` |

It's correct when:

- all three robots start moving together,
- each one slides in a straight line to its goal **without turning**, holding its starting heading,
- each one stops on its goal and stays parked while the others finish,
- the node logs `<robot> reached its goal (… mm out)` for each robot, then `task 2A complete`.

Check it against `/rN/odom`: each final position is within 20 mm of that robot's goal in `GOALS`.

> **`/odom` is not MuJoCo's frame.** The origin is the arena's **top-left** corner, `x` grows to the right and `y` grows **downwards**, both from `0` to `2.4384` m. Yaw is `0` along `+x` and grows clockwise on screen; all three robots start facing up, at `−π/2`.

## Files to edit

```
Software/workspace/src/task_2a/task_2a/path_follower.py
```

Paste in your Task 1B inverse kinematics and your Task 1C helpers, then fill in every `ADD YOUR CODE HERE` block: `wrap()`, `to_body()`, the `GoToPoint` PID controller and the marked parts of `RobotNode`. Set `CONTROL_HZ`, `GOAL_TOLERANCE` and `SETTLE_TICKS`. Leave the arena constants, `frame_check()` and `main()` as given.

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
ros2 launch hb_description task2a.launch.py

# Terminal 2 - your node (rebuild + source after every edit)
ros2 run task_2a path_follower
```

If the executable isn't found, the workspace isn't sourced — check with `ros2 pkg executables task_2a`.

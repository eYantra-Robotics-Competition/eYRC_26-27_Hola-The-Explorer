# Task 1B - Inverse Kinematics

## What to do

Derive and implement the **inverse kinematics** of the three-wheel holonomic drive: convert a desired body velocity (`vx`, `vy`, `ω`) into the three wheel speeds, ordered `[left, right, back]`.

Frame convention:

| Quantity | Meaning |
| --- | --- |
| `vx` | forward is positive |
| `vy` | left is positive |
| `ω` | counter-clockwise is positive (clockwise spin = negative `ω`) |

It's correct when each axis holds up independently in MuJoCo:

- pure forward/backward → straight line, heading unchanged
- pure sideways → clean **strafe**, heading fixed (a wrong matrix drifts diagonally or spins)
- pure rotation → spins in place, position essentially unchanged
- combined command → direction and turn rate both match

Verify against `/odom`, not just against what you commanded.


## Files to edit

```
Software/workspace/src/task_1b/task_1b/inverse_kinematics.py
```

One function is left blank, marked `# ----- YOUR CODE HERE -----`:

```python
body_velocity_to_wheel_speeds(vx, vy, w)  # returns [left, right, back]
```

Wheel radius, chassis radius and each wheel's angle are given at the top of the file. The ROS node and `cmd_vel_publisher` are already complete — don't change them.

## Run

```bash
cd ~/eYRC_26-27_Hola-The-Explorer
git pull
cd Software/workspace
colcon build
source install/setup.bash
```

Every new terminal needs its own `source install/setup.bash`, and every edit needs `colcon build && source install/setup.bash`.

Three terminals, **in this order**:

```bash
# Terminal 1 - simulation
ros2 launch hb_description task1b.launch.py

# Terminal 2 - your node (/cmd_vel -> /wheel_commands)
ros2 run task_1b inverse_kinematics

# Terminal 3 - test publisher
ros2 run task_1b cmd_vel_publisher
```

Before writing any code, you can drive the wheels by hand to see the problem:

```bash
ros2 topic pub --rate 10 /wheel_commands std_msgs/msg/Float64MultiArray "{data: [5.09, -5.09, 0.0]}"   # forward
ros2 topic pub --rate 10 /wheel_commands std_msgs/msg/Float64MultiArray "{data: [2.94, 2.94, -5.88]}"  # strafe left
ros2 topic pub --rate 10 /wheel_commands std_msgs/msg/Float64MultiArray "{data: [-1.26, -1.26, -1.26]}" # spin CW

ros2 topic echo /odom --field pose.pose.position
```
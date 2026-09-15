# Task 1C - PID Control & Shape Tracing

## What to do

Close the loop. Read the robot's pose from `/odom`, compare it against a goal pose, and let a **PID controller** (one each for `e_x`, `e_y`, `e_θ`) produce the velocity command — then push that through your Task 1B inverse kinematics to get wheel speeds.

The shape isn't yours to pick. A `/get_shape` service hands you one at random — `Circle`, `Square`, `Triangle`, `Rectangle` or `Pentagon` — with its center point and its size (radius, side length, or width/height). Call it as a **client**, build the waypoints from the response, and drive through them.

It's correct when:

- the client reads back the shape name and geometry correctly, and
- the path traced in `/odom` matches the assigned shape — straight edges and correct corners for a polygon, a smooth arc for a circle, with no corner-cutting, overshoot or spiralling.

Restart the node a few times to get different shapes; the controller has to handle all five, not just the first one you saw.


## Files to edit

```
Software/workspace/src/task_1c/task_1c/   # the boilerplate control script
```

Fill in the marked sections only: the `/get_shape` client call, the PID controller, and the call into your Task 1B inverse kinematics. Leave the node setup, subscribers, publishers and waypoint-building logic as given.

`Shape.msg` and `GetShape.srv` live inside `task_1c` itself, so import them straight from `task_1c` — no separate interface package.

## Run

```bash
cd ~/eYRC_26-27_Hola-The-Explorer
git pull
cd Software/workspace
colcon build
source install/setup.bash
```

```bash
# Terminal 1 - simulation (decor:=false for a lighter arena on slow machines)
ros2 launch hb_description task1c.launch.py decor:=true

# Terminal 2 - shape service (leave running)
ros2 run task_1c shape_service

# Terminal 3 - your node
ros2 run task_1c sim_control
```

Check the service by hand before writing any code:

```bash
ros2 service call /get_shape task_1c/srv/GetShape
ros2 pkg executables task_1c   # if the executable name isn't found
```

**Tuning order:** hardcode a single fixed goal pose first. Raise `Kp` until the response is decisive but not oscillating, add a small `Kd` to damp overshoot, then a small `Ki` to remove steady-state error. Only then wire in the service and chain across the full shape.
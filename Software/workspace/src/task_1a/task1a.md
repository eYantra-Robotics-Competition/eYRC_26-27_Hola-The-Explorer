# Task 1A - See the Checkpoint

## What to do

The arena has three checkpoints marked on the ground as **trapezoids** (cyan, green, orange). Read one frame from the overhead camera and reduce it to a `1280x720` **binary image** containing only the three trapezoid outlines in white, everything else black.

Output requirements:

| Check | How |
| --- | --- |
| Single channel, `1280x720` | `img.shape` is `(720, 1280)` |
| Truly binary | `np.unique(img)` is `[0 255]` |
| Exactly three outlines | Count contours, don't eyeball it |

Each trapezoid sits next to a vault of similar size, so "a quadrilateral of about this area" gives six shapes, not three.

**Optional (not graded):** find each trapezoid's centroid and pass it to the `/pixel_to_world` service to get real-world metres in the `/odom` frame. The arena is a 2.4384 m square, and `world_y` grows *downwards*.

## Install

```bash
sudo apt install libglfw3 libglfw3-dev python3-opencv

python3 -c "import cv2, numpy; print(cv2.__version__, numpy.__version__)"
# 4.5.4 1.21.5
```

> **Do not `pip install opencv-python`.** It pulls NumPy 2.x, and ROS 2 Humble is built against 1.x — `rclpy` then fails with `_ARRAY_API not found`.

## Files to edit

```
Software/workspace/src/task_1a/task_1a/camera_detection.py
```

Fill in the marked sections only; leave the node setup and service client as given.

## Run

```bash
cd ~/eYRC_26-27_Hola-The-Explorer
git pull
cd Software/workspace
colcon build
source install/setup.bash

# Terminal 1 - simulation
ros2 launch hb_description task1a.launch.py
# slow machine? -> ros2 launch hb_description task1a.launch.py decor:=false renderer:=gpu

# Terminal 2 - your node (rebuild + source after every edit)
ros2 run task_1a camera_detection
```

If the executable isn't found, the workspace isn't sourced — check with `ros2 pkg executables task_1a`.
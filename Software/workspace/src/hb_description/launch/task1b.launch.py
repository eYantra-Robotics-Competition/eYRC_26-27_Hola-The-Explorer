# Copyright (c) 2026 e-Yantra, IIT Bombay. All rights reserved.
# These simulation files and source code are the intellectual property of e-Yantra,
# IIT Bombay, provided solely for eYRC 2026-27 (Theme: Hola The Explorer).
# Sharing or redistribution of this material, in whole or in part, is not permitted.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "renderer",
                default_value="cpu",
                description="'cpu' renders via Mesa llvmpipe, which keeps the GPU out "
                "of the loop; 'gpu' uses the graphics driver.",
            ),
            DeclareLaunchArgument(
                "viewer_fps",
                default_value="60.0",
                description="Debug window refresh rate; 0 disables it (headless), which "
                "frees up rendering capacity for the camera.",
            ),
            DeclareLaunchArgument("camera_name", default_value="overhead_camera"),
            DeclareLaunchArgument("camera_width", default_value="1280"),
            DeclareLaunchArgument("camera_height", default_value="720"),
            DeclareLaunchArgument("camera_fps", default_value="30.0"),
            DeclareLaunchArgument(
                "stream_port",
                default_value="8080",
                description="TCP port the camera feed is served on. Open it with "
                "cv2.VideoCapture('http://127.0.0.1:<port>/stream').",
            ),
            DeclareLaunchArgument(
                "stream_quality",
                default_value="90",
                description="JPEG quality 1-100 for that feed. Lower trades image "
                "fidelity for bandwidth; 90 keeps marker edges clean.",
            ),
            DeclareLaunchArgument(
                "stream_bind",
                default_value="127.0.0.1",
                description="Address the feed listens on. Use 0.0.0.0 to reach it "
                "from another machine, or from the host when the simulation runs "
                "in a VM or WSL.",
            ),
            DeclareLaunchArgument(
                "decor",
                default_value="true",
                description="Draw the arena in its scenery. false draws the arena "
                "alone -- no sky, no ocean, no outer terrain -- which is ~6x cheaper "
                "on the CPU renderer. Visual only: physics, odometry and the camera "
                "intrinsics are identical either way.",
            ),
            DeclareLaunchArgument(
                "enable_camera",
                default_value="true",
                description="Launch the camera renderer alongside the bridge.",
            ),
            DeclareLaunchArgument(
                "enable_pixel_service",
                default_value="true",
                description="Start task_1a's pixel_to_world service, which converts "
                "overhead-camera pixels to the frame /odom is published in. Set false "
                "if task_1a is not built -- the simulation itself does not need it.",
            ),
            DeclareLaunchArgument(
                "enable_stream",
                default_value="true",
                description="Serve the camera feed as MJPEG over HTTP, so it opens with "
                "cv2.VideoCapture -- the same call a real USB camera takes.",
            ),
            Node(
                package="hb_description",
                executable="task1c_bridge",  # same world as Task 1C
                name="mujoco_robot_interface",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "renderer": LaunchConfiguration("renderer"),
                        "viewer_fps": ParameterValue(
                            LaunchConfiguration("viewer_fps"), value_type=float
                        ),
                        "decor": ParameterValue(
                            LaunchConfiguration("decor"), value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="hb_description",
                executable="task1c_camera",
                name="mujoco_camera_node",
                output="screen",
                # std::printf is block-buffered on a pipe, so without a pty the
                # camera's banner and any error it reports never reach the log.
                emulate_tty=True,
                # The camera reaches the bridge through the pose shared-memory
                # seqlock, not ROS -- it links no ROS libraries at all. It waits
                # for the bridge to create the segment, so ordering is handled.
                arguments=[
                    "--camera",
                    LaunchConfiguration("camera_name"),
                    "--width",
                    LaunchConfiguration("camera_width"),
                    "--height",
                    LaunchConfiguration("camera_height"),
                    "--fps",
                    LaunchConfiguration("camera_fps"),
                    "--renderer",
                    LaunchConfiguration("renderer"),
                    "--decor",
                    LaunchConfiguration("decor"),
                ],
                condition=IfCondition(LaunchConfiguration("enable_camera")),
            ),
            Node(
                package="hb_description",
                executable="camera_stream",
                name="camera_stream",
                output="screen",
                emulate_tty=True,
                # Reads the same frame shared memory the camera writes, and waits
                # for it, so ordering against the camera is handled. Width and
                # height must match the camera's: the shared block carries no
                # dimensions of its own, and a mismatch is rejected on startup.
                arguments=[
                    "--width",
                    LaunchConfiguration("camera_width"),
                    "--height",
                    LaunchConfiguration("camera_height"),
                    "--port",
                    LaunchConfiguration("stream_port"),
                    "--quality",
                    LaunchConfiguration("stream_quality"),
                    "--bind",
                    LaunchConfiguration("stream_bind"),
                ],
                condition=IfCondition(LaunchConfiguration("enable_stream")),
            ),
            Node(
                package="task_1a",
                executable="pixel_to_world_service",
                name="pixel_to_world_service",
                output="screen",
                # Must agree with what the camera actually renders: the pixel->metre
                # scale is derived from the image height and the camera's FOV, so a
                # mismatch here silently returns wrong coordinates.
                parameters=[
                    {
                        "image_width": ParameterValue(
                            LaunchConfiguration("camera_width"), value_type=int
                        ),
                        "image_height": ParameterValue(
                            LaunchConfiguration("camera_height"), value_type=int
                        ),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("enable_pixel_service")),
            ),
        ]
    )

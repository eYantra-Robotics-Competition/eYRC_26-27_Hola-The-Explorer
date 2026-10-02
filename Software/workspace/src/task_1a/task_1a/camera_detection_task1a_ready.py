#!/usr/bin/env python3

import math
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from shape_interface.srv import PixelToWorld


# ============================================================
# Task 1A - Hola The Explorer
# ============================================================

STREAM_URL = "http://127.0.0.1:8080/stream"
WINDOW = "camera_feed"
BINARY_WINDOW = "trapezoid_borders"
FPS_WINDOW = 30

# Arena crop for the default 1280x720 stream
ARENA_X0, ARENA_Y0, ARENA_X1, ARENA_Y1 = 304, 24, 975, 695

# Colour distance from the arena floor in LAB space
SAND_DISTANCE = 18

# Hough line parameters
HOUGH_THRESHOLD = 40
HOUGH_MIN_LENGTH = 35
HOUGH_MAX_GAP = 25

# Trapezoid filters
MIN_TRAPEZOID_AREA = 1200
PARALLEL_TOLERANCE_DEG = 7

# ROS reporting interval
REPORT_PERIOD_SEC = 0.5


def angle_difference(a, b):
    """
    Return the smallest difference between two line angles.
    Angles are treated modulo 180 degrees because a line at 0°
    is the same direction as a line at 180°.
    """
    diff = abs(a - b)
    return min(diff, 180.0 - diff)


def centre_of_quad(corners):
    """
    Find the area centroid of a quadrilateral using OpenCV moments.
    """

    contour = np.asarray(corners, dtype=np.float32).reshape((-1, 1, 2))

    moments = cv2.moments(contour)

    if abs(moments["m00"]) < 1e-9:
        return 0.0, 0.0

    cx = moments["m10"] / moments["m00"]
    cy = moments["m01"] / moments["m00"]

    return float(cx), float(cy)


def find_trapezoids(frame):
    """
    Detect the station funnels in one camera frame.

    Returns:
        binary:
            Full-frame binary image containing only accepted trapezoid borders.
        trapezoids:
            List of (centre_x, centre_y, corners), where corners are in
            full-frame pixel coordinates.
    """

    binary = np.zeros(frame.shape[:2], dtype=np.uint8)
    trapezoids = []

    # --------------------------------------------------------
    # 1. Crop to the arena
    # --------------------------------------------------------
    arena = frame[ARENA_Y0:ARENA_Y1, ARENA_X0:ARENA_X1].copy()

    if arena.size == 0:
        return binary, trapezoids

    # --------------------------------------------------------
    # 2. Convert BGR -> LAB
    # --------------------------------------------------------
    lab = cv2.cvtColor(arena, cv2.COLOR_BGR2LAB)

    # --------------------------------------------------------
    # 3. Find the most common LAB colour of the floor
    # --------------------------------------------------------
    floor_colour = []

    for channel in cv2.split(lab):
        histogram = np.bincount(channel.ravel(), minlength=256)
        floor_colour.append(int(np.argmax(histogram)))

    floor_colour = np.asarray(floor_colour, dtype=np.int16)

    # --------------------------------------------------------
    # 4. Measure every pixel's distance from floor colour
    # --------------------------------------------------------
    diff = lab.astype(np.int16) - floor_colour

    distance = np.sqrt(
        np.sum(diff.astype(np.float32) ** 2, axis=2)
    )

    # --------------------------------------------------------
    # 5. Create "not floor" mask
    # --------------------------------------------------------
    mask = (distance > SAND_DISTANCE).astype(np.uint8) * 255

    # --------------------------------------------------------
    # 6. Clean mask with morphological closing
    # --------------------------------------------------------
    mask_kernel = np.ones((3, 3), dtype=np.uint8)

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        mask_kernel
    )

    # --------------------------------------------------------
    # 7. Canny edge detection
    # --------------------------------------------------------
    edges = cv2.Canny(mask, 50, 150)

    # --------------------------------------------------------
    # 8. Probabilistic Hough line detection
    # --------------------------------------------------------
    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=HOUGH_THRESHOLD,
        minLineLength=HOUGH_MIN_LENGTH,
        maxLineGap=HOUGH_MAX_GAP
    )

    if lines is None:
        return binary, trapezoids

    # --------------------------------------------------------
    # 9. Draw detected line segments on a temporary image
    # --------------------------------------------------------
    scratch = np.zeros_like(mask)

    for line in lines:
        x1, y1, x2, y2 = map(int, line[0])

        cv2.line(
            scratch,
            (x1, y1),
            (x2, y2),
            255,
            3
        )

    # --------------------------------------------------------
    # 10. Close gaps between line segments
    # --------------------------------------------------------
    scratch_kernel = np.ones((5, 5), dtype=np.uint8)

    scratch = cv2.morphologyEx(
        scratch,
        cv2.MORPH_CLOSE,
        scratch_kernel
    )

    # --------------------------------------------------------
    # 11. Find contours using RETR_CCOMP
    # --------------------------------------------------------
    contours, hierarchy = cv2.findContours(
        scratch,
        cv2.RETR_CCOMP,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if hierarchy is None:
        return binary, trapezoids

    hierarchy = hierarchy[0]

    # --------------------------------------------------------
    # 12. Examine every contour
    # --------------------------------------------------------
    candidates = []

    for i, contour in enumerate(contours):

        # A closed funnel becomes an inner contour.
        # Inner contour => has a parent.
        parent = int(hierarchy[i][3])

        if parent < 0:
            continue

        # Area filter
        area = abs(cv2.contourArea(contour))

        if area < MIN_TRAPEZOID_AREA:
            continue

        # Polygon approximation
        perimeter = cv2.arcLength(contour, True)

        if perimeter <= 0:
            continue

        approx = cv2.approxPolyDP(
            contour,
            0.03 * perimeter,
            True
        )

        # We need exactly four vertices
        if len(approx) != 4:
            continue

        # Must be convex
        if not cv2.isContourConvex(approx):
            continue

        # ----------------------------------------------------
        # 13. Extract the four corners
        # ----------------------------------------------------
        points = approx.reshape(4, 2).astype(np.float32)

        # ----------------------------------------------------
        # 14. Calculate the direction of each side
        # ----------------------------------------------------
        angles = []

        for j in range(4):
            p1 = points[j]
            p2 = points[(j + 1) % 4]

            dx = float(p2[0] - p1[0])
            dy = float(p2[1] - p1[1])

            angle = math.degrees(math.atan2(dy, dx))
            angle %= 180.0

            angles.append(angle)

        # ----------------------------------------------------
        # 15. A trapezoid has exactly ONE pair of parallel
        #     opposite sides.
        # ----------------------------------------------------
        pair_02_parallel = (
            angle_difference(angles[0], angles[2])
            <= PARALLEL_TOLERANCE_DEG
        )

        pair_13_parallel = (
            angle_difference(angles[1], angles[3])
            <= PARALLEL_TOLERANCE_DEG
        )

        if int(pair_02_parallel) + int(pair_13_parallel) != 1:
            continue

        # ----------------------------------------------------
        # 16. Convert crop coordinates -> full-frame coordinates
        # ----------------------------------------------------
        corners = points.copy()

        corners[:, 0] += ARENA_X0
        corners[:, 1] += ARENA_Y0

        # ----------------------------------------------------
        # 17. Calculate area centroid
        # ----------------------------------------------------
        cx, cy = centre_of_quad(corners)

        # Store candidate
        candidates.append(
            (cx, cy, corners, area)
        )

    # --------------------------------------------------------
    # 18. Remove duplicate detections
    # --------------------------------------------------------
    # Depending on Hough/contour conditions, the same funnel can
    # occasionally produce two very close detections.
    candidates.sort(key=lambda item: item[3], reverse=True)

    for cx, cy, corners, area in candidates:

        duplicate = False

        for old_cx, old_cy, _, _ in trapezoids:
            if math.hypot(cx - old_cx, cy - old_cy) < 25.0:
                duplicate = True
                break

        if duplicate:
            continue

        trapezoids.append(
            (cx, cy, corners)
        )

    # --------------------------------------------------------
    # 19. Draw ONLY accepted trapezoid borders in binary image
    # --------------------------------------------------------
    for cx, cy, corners in trapezoids:
        polygon = np.round(corners).astype(np.int32)

        cv2.polylines(
            binary,
            [polygon],
            isClosed=True,
            color=255,
            thickness=2
        )

    return binary, trapezoids


def main():
    """
    Run the camera stream, detect trapezoids and convert their
    pixel centres to world coordinates through ROS.
    """

    rclpy.init()

    node = Node("camera_feed")

    client = node.create_client(
        PixelToWorld,
        "pixel_to_world"
    )

    node.get_logger().info(
        "waiting for the pixel_to_world service ..."
    )

    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(
            "pixel_to_world is not up. "
            "Start it first: ros2 run task_1a pixel_to_world_service"
        )
        rclpy.shutdown()
        return

    # --------------------------------------------------------
    # Open camera stream
    # --------------------------------------------------------
    cap = cv2.VideoCapture(STREAM_URL)

    if not cap.isOpened():
        node.get_logger().error(
            f"could not open {STREAM_URL}"
        )
        node.get_logger().error(
            "start the simulation first: "
            "ros2 launch hb_description task1a.launch.py"
        )
        rclpy.shutdown()
        return

    stamps = deque(maxlen=FPS_WINDOW)

    fps = 0.0
    last_report = 0.0

    try:
        while rclpy.ok():

            ok, frame = cap.read()

            if not ok:
                node.get_logger().error(
                    "camera frame could not be read"
                )
                break

            # ------------------------------------------------
            # FPS calculation
            # ------------------------------------------------
            stamps.append(time.monotonic())

            if len(stamps) >= 2:
                span = stamps[-1] - stamps[0]

                if span > 0:
                    fps = (len(stamps) - 1) / span
                else:
                    fps = 0.0

            # ------------------------------------------------
            # Detect trapezoids
            # ------------------------------------------------
            binary, trapezoids = find_trapezoids(frame)

            # ------------------------------------------------
            # Draw detection overlay
            # ------------------------------------------------
            for cx, cy, corners in trapezoids:

                polygon = np.round(
                    corners
                ).astype(np.int32)

                cv2.polylines(
                    frame,
                    [polygon],
                    True,
                    (0, 0, 255),
                    2
                )

                cv2.circle(
                    frame,
                    (
                        int(round(cx)),
                        int(round(cy))
                    ),
                    6,
                    (0, 255, 255),
                    -1
                )

            # ------------------------------------------------
            # Display status
            # ------------------------------------------------
            cv2.putText(
                frame,
                f"{fps:5.1f} FPS",
                (12, 34),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )

            cv2.putText(
                frame,
                f"{len(trapezoids)} trapezoids",
                (12, 68),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )

            cv2.imshow(WINDOW, frame)
            cv2.imshow(BINARY_WINDOW, binary)

            # ------------------------------------------------
            # ROS pixel -> world conversion
            # ------------------------------------------------
            now = time.monotonic()

            if trapezoids and (
                now - last_report >= REPORT_PERIOD_SEC
            ):
                last_report = now

                print(
                    f"\n{len(trapezoids)} trapezoid(s):"
                )

                # Stable order: top-to-bottom, then left-to-right
                ordered = sorted(
                    trapezoids,
                    key=lambda item: (item[1], item[0])
                )

                for cx, cy, _ in ordered:

                    request = PixelToWorld.Request()

                    request.pixel_x = float(cx)
                    request.pixel_y = float(cy)

                    # IMPORTANT:
                    # Use asynchronous service call.
                    future = client.call_async(request)

                    # Wait for result without using the
                    # blocking client.call() method.
                    rclpy.spin_until_future_complete(
                        node,
                        future,
                        timeout_sec=1.0
                    )

                    result = future.result()

                    if result is None:

                        print(
                            f"  pixel ({cx:7.2f}, {cy:7.2f})  ->  "
                            f"timeout"
                        )

                    elif not result.success:

                        print(
                            f"  pixel ({cx:7.2f}, {cy:7.2f})  ->  "
                            f"ERROR: {result.message}"
                        )

                    else:

                        wx = float(result.world_x)
                        wy = float(result.world_y)

                        print(
                            f"  pixel ({cx:7.2f}, {cy:7.2f})  ->  "
                            f"world ({wx:6.3f}, {wy:6.3f}) m"
                        )

            # ------------------------------------------------
            # Press q to quit
            # ------------------------------------------------
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

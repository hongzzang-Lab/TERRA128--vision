"""
TERRA128 Vision Module
-----------------------------------
핵심 Vision Pipeline만 정리한 요약 코드.

기능:
- RealSense RGB-D 입력
- IMU Roll/Pitch 보정
- Point Cloud → BEV 변환
- Terrain Cost 계산
- A* 기반 경로 탐색
- AR Overlay 시각화
"""

import cv2
import math
import heapq
import socket
import numpy as np
import pyrealsense2 as rs


# ============================================================
# Configuration
# ============================================================
UDP_IP = "0.0.0.0"
UDP_PORT = 5007

CAMERA_HEIGHT = 0.27
VOXEL_SIZE = 0.1

Z_MIN, Z_MAX = 0.4, 3.2
X_MIN, X_MAX = -1.4, 1.4

ROWS = int((Z_MAX - Z_MIN) / VOXEL_SIZE)
COLS = int((X_MAX - X_MIN) / VOXEL_SIZE)


# ============================================================
# IMU UDP Receiver
# ============================================================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)
sock.bind((UDP_IP, UDP_PORT))


# ============================================================
# RealSense Initialization
# ============================================================
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipeline.start(config)
pc = rs.pointcloud()


# ============================================================
# Utility Functions
# ============================================================
def get_rotation_matrix(roll_deg, pitch_deg):
    """IMU Roll/Pitch 기반 회전 보정"""

    roll = math.radians(roll_deg)
    pitch = math.radians(-pitch_deg)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(pitch), -np.sin(pitch)],
        [0, np.sin(pitch), np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(roll), -np.sin(roll), 0],
        [np.sin(roll), np.cos(roll), 0],
        [0, 0, 1]
    ])

    return np.dot(Rz, Rx)


def calculate_cell_cost(height_diff, slope_deg):
    """지형 위험도 계산"""

    if slope_deg > 18:
        return 255

    cost = 0

    if slope_deg > 10:
        cost += int((slope_deg - 10) * 10)

    if height_diff > 0.05:
        return 255

    elif height_diff > 0.02:
        cost += 120

    return min(255, cost)


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def a_star(cost_map, start, goal):
    """Costmap 기반 A* 경로 탐색"""

    frontier = []
    heapq.heappush(frontier, (0, start))

    came_from = {start: None}
    cost_so_far = {start: 0}

    directions = [
        (0, 1), (1, 0),
        (0, -1), (-1, 0)
    ]

    while frontier:
        _, current = heapq.heappop(frontier)

        if current == goal:
            break

        for dx, dy in directions:
            nxt = (current[0] + dx, current[1] + dy)

            r, c = nxt

            if 0 <= r < ROWS and 0 <= c < COLS:
                cell_cost = cost_map[r, c]

                if cell_cost == 255:
                    continue

                new_cost = cost_so_far[current] + 1 + (cell_cost / 50)

                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost

                    priority = new_cost + heuristic(goal, nxt)
                    heapq.heappush(frontier, (priority, nxt))

                    came_from[nxt] = current

    return came_from


# ============================================================
# Main Loop
# ============================================================
print("[TERRA128] Vision Module Started")

try:
    while True:

        # ----------------------------------------------------
        # IMU Data Receive
        # ----------------------------------------------------
        roll, pitch = 0.0, 0.0

        try:
            data, _ = sock.recvfrom(1024)
            roll, pitch = map(float, data.decode().split(','))
        except:
            pass

        # ----------------------------------------------------
        # RealSense Frame
        # ----------------------------------------------------
        frames = pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        color_img = np.asanyarray(color_frame.get_data())

        # ----------------------------------------------------
        # Point Cloud Generation
        # ----------------------------------------------------
        verts = np.asanyarray(
            pc.calculate(depth_frame).get_vertices()
        ).view(np.float32).reshape(-1, 3)

        # ----------------------------------------------------
        # IMU Compensation
        # ----------------------------------------------------
        R = get_rotation_matrix(roll, pitch)
        corrected = np.dot(verts, R.T)

        # ----------------------------------------------------
        # BEV Costmap
        # ----------------------------------------------------
        cost_map = np.zeros((ROWS, COLS), dtype=np.uint8)

        for point in corrected:

            x, y, z = point

            if not (Z_MIN < z < Z_MAX):
                continue

            if not (X_MIN < x < X_MAX):
                continue

            row = int((Z_MAX - z) / VOXEL_SIZE)
            col = int((x - X_MIN) / VOXEL_SIZE)

            height_diff = abs(CAMERA_HEIGHT - y)
            slope_deg = abs(y * 30)

            cost = calculate_cell_cost(height_diff, slope_deg)

            cost_map[row, col] = max(cost_map[row, col], cost)

        # ----------------------------------------------------
        # Path Planning
        # ----------------------------------------------------
        start = (ROWS - 1, COLS // 2)
        goal = (5, COLS // 2)

        path = a_star(cost_map, start, goal)

        # ----------------------------------------------------
        # Visualization
        # ----------------------------------------------------
        overlay = color_img.copy()

        danger_mask = cost_map == 255

        cv2.putText(
            overlay,
            f"ROLL:{roll:.1f}  PITCH:{pitch:.1f}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        cv2.imshow("TERRA128 Vision", overlay)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

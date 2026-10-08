"""Frontier detection for occupancy grids (no ROS dependencies, easy to test).

Grid convention (same as nav_msgs/OccupancyGrid after reshaping to (height, width)):
    -1        unknown
    0..100    probability the cell is occupied (0 = free)
A frontier cell is a free cell with at least one unknown cell among its 8 neighbours,
i.e. the edge of the explored region.
"""

import math
from collections import deque

import numpy as np


def frontier_mask(grid, free_max=25):
    """Boolean array marking free cells that touch unknown space."""
    unknown = grid < 0
    free = (grid >= 0) & (grid <= free_max)
    height, width = grid.shape

    padded = np.pad(unknown, 1, constant_values=False)
    near_unknown = np.zeros_like(unknown)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            near_unknown |= padded[1 + dy : 1 + dy + height, 1 + dx : 1 + dx + width]
    return free & near_unknown


def cluster_cells(mask):
    """Group True cells of a mask into 8-connected clusters of (x, y) cells."""
    height, width = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    clusters = []
    for y, x in zip(*np.nonzero(mask)):
        x, y = int(x), int(y)
        if seen[y, x]:
            continue
        seen[y, x] = True
        queue = deque([(x, y)])
        cells = []
        while queue:
            cx, cy = queue.popleft()
            cells.append((cx, cy))
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    nx, ny = cx + dx, cy + dy
                    if (
                        0 <= nx < width
                        and 0 <= ny < height
                        and mask[ny, nx]
                        and not seen[ny, nx]
                    ):
                        seen[ny, nx] = True
                        queue.append((nx, ny))
        clusters.append(cells)
    return clusters


def find_frontiers(grid, free_max=25, min_size=8):
    """Frontier clusters with at least min_size cells (smaller ones are noise)."""
    clusters = cluster_cells(frontier_mask(grid, free_max))
    return [c for c in clusters if len(c) >= min_size]


def cluster_target(cells):
    """The cluster cell closest to the cluster's centre (always a real free cell)."""
    cx = sum(x for x, _ in cells) / len(cells)
    cy = sum(y for _, y in cells) / len(cells)
    return min(cells, key=lambda c: (c[0] - cx) ** 2 + (c[1] - cy) ** 2)


def choose_frontier(
    clusters, robot_cell, blacklist=(), blacklist_radius=10, min_distance=6
):
    """Pick the nearest usable frontier target cell, or None if there is none.

    Targets within blacklist_radius cells of a blacklisted (previously failed) goal, or
    closer than min_distance cells to the robot, are skipped.
    """
    best = None
    for cells in clusters:
        target = cluster_target(cells)
        distance = math.hypot(target[0] - robot_cell[0], target[1] - robot_cell[1])
        if distance < min_distance:
            continue
        if any(
            math.hypot(target[0] - bx, target[1] - by) <= blacklist_radius
            for bx, by in blacklist
        ):
            continue
        if best is None or distance < best[0]:
            best = (distance, target)
    return None if best is None else best[1]

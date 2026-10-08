import numpy as np

from autonomous_explorer.frontier import (
    choose_frontier,
    cluster_target,
    find_frontiers,
    frontier_mask,
)


def test_fully_explored_room_has_no_frontiers():
    grid = np.zeros((20, 20), dtype=int)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = 100  # walls all round
    assert find_frontiers(grid) == []


def test_boundary_between_free_and_unknown_is_one_frontier():
    grid = np.full((20, 20), -1, dtype=int)
    grid[:, 10:] = 0  # right half explored and free, left half unknown
    clusters = find_frontiers(grid, min_size=5)
    assert len(clusters) == 1
    assert len(clusters[0]) == 20  # one frontier cell per row
    assert all(x == 10 for x, _ in clusters[0])


def test_wall_between_free_and_unknown_is_not_a_frontier():
    grid = np.full((20, 20), -1, dtype=int)
    grid[:, 11:] = 0
    grid[:, 10] = 100  # solid wall separates explored area from unknown
    assert find_frontiers(grid, min_size=1) == []


def test_tiny_speckles_are_ignored():
    grid = np.zeros((20, 20), dtype=int)
    grid[10, 10] = -1  # one unknown cell inside a free room
    assert len(find_frontiers(grid, min_size=1)) == 1  # found without the size filter
    assert find_frontiers(grid, min_size=20) == []  # ignored as noise


def test_unknown_cells_and_occupied_cells_are_never_frontier_cells():
    grid = np.full((10, 10), -1, dtype=int)
    grid[:, 5:] = 0
    grid[3, 5] = 100
    mask = frontier_mask(grid)
    assert not mask[grid < 0].any()
    assert not mask[grid == 100].any()


def test_cluster_target_is_a_real_cell_of_the_cluster():
    cells = [(5, y) for y in range(10)]
    assert cluster_target(cells) in cells


def test_choose_nearest_frontier():
    near = [(2, y) for y in range(10)]
    far = [(18, y) for y in range(10)]
    target = choose_frontier([far, near], robot_cell=(0, 5), min_distance=1)
    assert target in near


def test_blacklisted_frontier_is_skipped():
    near = [(2, y) for y in range(10)]
    far = [(18, y) for y in range(10)]
    first = choose_frontier([near, far], robot_cell=(0, 5), min_distance=1)
    second = choose_frontier(
        [near, far],
        robot_cell=(0, 5),
        blacklist=[first],
        blacklist_radius=3,
        min_distance=1,
    )
    assert first in near
    assert second in far


def test_no_usable_frontier_returns_none():
    only = [(2, y) for y in range(10)]
    assert (
        choose_frontier(
            [only],
            robot_cell=(0, 5),
            blacklist=[(2, 5)],
            blacklist_radius=10,
            min_distance=1,
        )
        is None
    )
    assert choose_frontier([], robot_cell=(0, 0)) is None

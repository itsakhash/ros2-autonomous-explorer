import math

import numpy as np

from autonomous_explorer.frontier_explorer import pick_seed_cell


def blank(h=21, w=21, value=-1):
    return np.full((h, w), value, dtype=np.int16)


def test_picks_farthest_free_cell_within_range():
    grid = blank(value=0)  # everything free
    cell = pick_seed_cell(grid, (10, 10), free_max=25, min_dist=5, max_dist=8)
    assert cell is not None
    assert math.hypot(cell[0] - 10, cell[1] - 10) == 8.0


def test_returns_none_when_nothing_is_far_enough():
    grid = blank()
    grid[8:13, 8:13] = 0  # free only within 2-3 cells of the robot
    assert pick_seed_cell(grid, (10, 10), 25, min_dist=5, max_dist=8) is None


def test_returns_none_for_unknown_or_occupied_maps():
    assert pick_seed_cell(blank(value=-1), (10, 10), 25, 5, 8) is None
    assert pick_seed_cell(blank(value=100), (10, 10), 25, 5, 8) is None


def test_returns_x_then_y():
    grid = blank()
    grid[10, 16] = 0  # row (y) 10, column (x) 16: six cells to the right
    assert pick_seed_cell(grid, (10, 10), 25, min_dist=5, max_dist=8) == (16, 10)


def test_respects_free_max():
    grid = blank()
    grid[10, 15] = 30  # too uncertain to count as free
    grid[10, 5] = 25  # exactly at the limit: still free
    assert pick_seed_cell(grid, (10, 10), 25, min_dist=3, max_dist=8) == (5, 10)


def test_skips_blacklisted_cells():
    grid = blank()
    grid[10, 15] = 0  # 5 cells away
    grid[10, 4] = 0  # 6 cells away: the farther one, normally chosen
    args = dict(free_max=25, min_dist=3, max_dist=8)
    assert pick_seed_cell(grid, (10, 10), **args) == (4, 10)
    assert pick_seed_cell(
        grid, (10, 10), blacklist=[(4, 10)], blacklist_radius=1.0, **args
    ) == (15, 10)
    assert (
        pick_seed_cell(
            grid,
            (10, 10),
            blacklist=[(4, 10), (15, 10)],
            blacklist_radius=1.0,
            **args,
        )
        is None
    )

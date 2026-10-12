"""Cold bench for the obstacle cost gradient: known geometry in -> known route out.

No robot, no stack. Every case is a synthetic grid at the real 0.05 m
resolution, so the numbers are metres a tape measure would agree with.

The three things the gradient has to get right at once:
  1. given two ways round, take the wide one;
  2. given open space beside a wall, keep away from the wall;
  3. given a tight gap as the ONLY way through, still go through it. A lethal
     inflation rounded up to whole cells walls off doorways the body fits
     through, and the robot then explores nothing.

The corridor and gap geometry is built for a 0.50 m wide REFERENCE body (the
body the cost field was designed and swept on). Those cases pass its radii
explicitly, so they do not depend on the robot profile. The last case checks
the profile path: called with no radii, the field takes MOSS's clearances
from moss_dimos.robot_profile, read at call time.

Needs dimOS. Run from the repository root:
    python benches/bench_cost_gradient_cold.py
"""

import os
import sys
from pathlib import Path as _FsPath

import numpy as np
from scipy import ndimage

sys.path.insert(0, str(_FsPath(__file__).resolve().parent.parent))

from dimos.msgs.geometry_msgs.Pose import Pose  # noqa: E402
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid  # noqa: E402
from moss_dimos.recovering_planner import min_cost_astar  # noqa: E402  (dimOS's, from wherever this dimOS keeps it)
from moss_dimos import robot_profile  # noqa: E402
from moss_dimos.recovering_planner import (  # noqa: E402
    PIVOT_PENALTY,
    clearance_cost_map,
    clearances,
)

RES = 0.05

# The reference body: width, pivot envelope and the cross-track margin the
# follower may spend. Lethal = width/2 + margin, pivot = pivot diameter/2,
# the same formulas as robot_profile.derived().
REF_ROBOT_WIDTH_M = 0.50
REF_PIVOT_DIAMETER_M = 0.78
REF_CONTROL_MARGIN_M = 0.05
REF_LETHAL_CLEARANCE_M = REF_ROBOT_WIDTH_M / 2 + REF_CONTROL_MARGIN_M   # 0.30
REF_PIVOT_CLEARANCE_M = REF_PIVOT_DIAMETER_M / 2                        # 0.39
REF = {"lethal_m": REF_LETHAL_CLEARANCE_M, "pivot_m": REF_PIVOT_CLEARANCE_M}

# MOSS nominal body (robot_profile.MOSS): 0.34 m wide, 0.50 m pivot diameter,
# 0.05 m margin -> lethal 0.22 m, pivot 0.25 m.
MOSS_LETHAL_M = 0.34 / 2 + 0.05
MOSS_PIVOT_M = 0.50 / 2

# env vars that would change the profile under the bench; cleared while the
# profile case runs and restored after it
_PROFILE_ENV = ("MOSS_ROBOT_WIDTH_M", "MOSS_PIVOT_DIAMETER_M", "MOSS_CONTROL_MARGIN_M")


def make_grid(cells: np.ndarray) -> OccupancyGrid:
    origin = Pose()
    origin.position.x, origin.position.y, origin.position.z = 0.0, 0.0, 0.0
    origin.orientation.w = 1.0
    return OccupancyGrid(
        grid=cells.astype(np.int8), resolution=RES, origin=origin, frame_id="map", ts=0.0
    )


def world(gx: int, gy: int) -> tuple[float, float]:
    return ((gx + 0.5) * RES, (gy + 0.5) * RES)


def path_cells(path, shape) -> list[tuple[int, int]]:
    out = []
    for pose in path.poses:
        gx, gy = int(pose.position.x / RES), int(pose.position.y / RES)
        if 0 <= gy < shape[0] and 0 <= gx < shape[1]:
            out.append((gx, gy))
    return out


def clearance_field(cells: np.ndarray) -> np.ndarray:
    """Metres from every cell to the nearest occupied cell."""
    return ndimage.distance_transform_edt(cells < CostValues.OCCUPIED) * RES


def usable_cells(cells: np.ndarray, row: int, lo: int, hi: int) -> int:
    """Cells of that corridor the reference body may actually stand on."""
    field = clearance_field(cells)
    return int(np.sum(field[row, lo:hi] >= REF_LETHAL_CLEARANCE_M))


def gap_crossing(gap_m: float, **radii) -> tuple[int, int, bool]:
    """One wall across the grid with a single gap: (gap cells, passable cells
    on the wall row, does the planned path cross the wall row)."""
    gap = int(round(gap_m / RES))
    height, width = 61, 41
    cells = np.zeros((height, width), dtype=np.int8)
    wall_row = height // 2
    x0 = (width - gap) // 2
    cells[wall_row, :x0] = CostValues.OCCUPIED
    cells[wall_row, x0 + gap:] = CostValues.OCCUPIED

    costmap = clearance_cost_map(make_grid(cells), **radii)
    passable = int(np.sum(costmap.grid[wall_row] < CostValues.OCCUPIED))
    path = min_cost_astar(costmap, world(width // 2, height - 3), world(width // 2, 2))
    crossed = path is not None and any(gy == wall_row for gx, gy in path_cells(path, cells.shape))
    return gap, passable, crossed


def test_constants_are_physically_ordered() -> None:
    """The three radii must stay in the order the design assumes: the body
    fits, the margin is real, and pivoting needs more room than driving."""
    assert REF_ROBOT_WIDTH_M / 2 < REF_LETHAL_CLEARANCE_M < REF_PIVOT_CLEARANCE_M, (
        REF_ROBOT_WIDTH_M / 2,
        REF_LETHAL_CLEARANCE_M,
        REF_PIVOT_CLEARANCE_M,
    )
    print(
        f"  reference body/2 {REF_ROBOT_WIDTH_M / 2:.3f} < lethal {REF_LETHAL_CLEARANCE_M:.3f} "
        f"< pivot {REF_PIVOT_CLEARANCE_M:.3f} m"
    )


def test_prefers_the_wide_corridor() -> None:
    """Two parallel corridors of the same length, both joining start to goal.

    Sized so that after the lethal radius one leaves 3 usable cells and the
    other 8, counted in cells the planner can really stand on. (3 and 8 raw
    cells would be 0.15 m and 0.40 m, both narrower than the 0.50 m reference
    body, so neither would be a corridor at all.)
    """
    height, width = 101, 121
    cells = np.zeros((height, width), dtype=np.int8)
    band_lo, band_hi = 25, 76           # the corridor band; rooms above and below
    narrow_lo, narrow_hi = 8, 21        # 13 cells = 0.65 m between the faces
    wide_lo, wide_hi = 34, 52           # 18 cells = 0.90 m between the faces
    cells[band_lo:band_hi, :] = CostValues.OCCUPIED
    cells[band_lo:band_hi, narrow_lo:narrow_hi] = 0
    cells[band_lo:band_hi, wide_lo:wide_hi] = 0

    mid = height // 2
    n_narrow = usable_cells(cells, mid, narrow_lo, narrow_hi)
    n_wide = usable_cells(cells, mid, wide_lo, wide_hi)
    print(f"  usable width: narrow 0.65 m -> {n_narrow} cells, wide 0.90 m -> {n_wide} cells")
    assert (n_narrow, n_wide) == (3, 8), (n_narrow, n_wide)

    # start and goal sit between the two corridor mouths, so neither route is
    # shorter: narrow centre x=14, wide centre x=42.5, start x=28.
    start = world(28, 10)
    goal = world(28, height - 11)
    costmap = clearance_cost_map(make_grid(cells), **REF)
    path = min_cost_astar(costmap, goal, start)
    assert path is not None and path.poses, "no path through either corridor"

    crossing = [gx for gx, gy in path_cells(path, cells.shape) if band_lo + 10 <= gy < band_hi - 10]
    assert crossing, "path never crossed the corridor band"
    mean_x = float(np.mean(crossing))
    took_wide = mean_x > (narrow_hi + wide_lo) / 2
    print(
        f"  narrow centre x={(narrow_lo + narrow_hi) / 2 * RES:.2f} m, "
        f"wide centre x={(wide_lo + wide_hi) / 2 * RES:.2f} m, "
        f"path crossed at x={mean_x * RES:.2f} m -> {'WIDE' if took_wide else 'NARROW'}"
    )
    assert took_wide, "path took the narrow corridor with a wide one available"


def test_the_penalty_is_what_chooses_the_wide_corridor() -> None:
    """Pin the constant to the behaviour it buys.

    With the penalty off, dimOS's ridge cost alone is indifferent: it puts cost
    0 on the medial axis of BOTH corridors, so the tie falls to length and the
    narrow one wins. This is the measurement that justifies PIVOT_PENALTY
    existing at all - if this test ever passes at penalty 0, the term is dead
    weight and should go.
    """
    height, width = 101, 121
    cells = np.zeros((height, width), dtype=np.int8)
    cells[25:76, :] = CostValues.OCCUPIED
    cells[25:76, 8:21] = 0          # 0.65 m
    cells[25:76, 34:52] = 0         # 0.90 m
    grid = make_grid(cells)
    start, goal = world(28, 10), world(28, height - 11)

    for penalty, expected in ((0, "NARROW"), (PIVOT_PENALTY, "WIDE")):
        path = min_cost_astar(clearance_cost_map(grid, penalty=penalty, **REF), goal, start)
        assert path is not None and path.poses, f"no path at penalty {penalty}"
        crossing = [gx for gx, gy in path_cells(path, cells.shape) if 35 <= gy < 66]
        got = "WIDE" if float(np.mean(crossing)) > 27 else "NARROW"
        print(f"  penalty {penalty:>4} -> {got}")
        assert got == expected, f"penalty {penalty} took the {got} corridor"


def test_keeps_away_from_a_lone_wall() -> None:
    """One wall with open space beside it: the path must stop hugging it.

    Start and goal sit 0.325 m off the wall - passable, but closer than the
    0.39 m reference pivot radius. Without a gradient A* runs straight along
    that line. With one it has to bow away wherever there is room; the ends
    are pinned by the start and goal, so the >= 0.35 m rule is checked on the
    middle.
    """
    height, width = 61, 121
    cells = np.zeros((height, width), dtype=np.int8)
    cells[0, :] = CostValues.OCCUPIED          # the wall, along y = 0
    grid = make_grid(cells)
    field = clearance_field(cells)

    start, goal = world(6, 6), world(width - 7, 6)   # 0.325 m off the wall: passable, under the pivot radius
    flat = min_cost_astar(grid, goal, start)                 # no gradient at all
    graded = min_cost_astar(clearance_cost_map(grid, **REF), goal, start)
    assert flat is not None and graded is not None

    def stats(path):
        pts = path_cells(path, cells.shape)
        keep = pts[len(pts) // 6 : -len(pts) // 6]           # drop the pinned ends
        vals = [field[gy, gx] for gx, gy in keep]
        return float(np.mean(vals)), float(np.min(vals))

    flat_mean, flat_min = stats(flat)
    graded_mean, graded_min = stats(graded)
    print(
        f"  middle-of-path clearance: no gradient mean {flat_mean:.3f} min {flat_min:.3f} m"
        f"  ->  gradient mean {graded_mean:.3f} min {graded_min:.3f} m"
    )
    assert graded_min >= 0.35, f"path came within {graded_min:.3f} m of the wall"
    assert graded_min > flat_min, "the gradient bought no clearance at all"


def test_still_takes_a_tight_gap_when_it_is_the_only_way() -> None:
    """A gap and nothing else. The reference body is 0.50 m wide, so 0.55 m
    must go through (11 free cells: the wall cells on either side are 0.60 m
    apart, centre to centre, and the middle cell sits exactly ON the lethal
    radius).

    Regression guard. dimOS's own 1.1x inflation walls these off - every gap
    up to 0.62 m plans "no path" with it - and the robot is then left in
    front of doorways it fits through.
    """
    for gap_m, must_pass in (
        (0.40, False), (0.45, False), (0.50, False),
        (0.55, True), (0.62, True), (0.80, True),
    ):
        gap, passable, crossed = gap_crossing(gap_m, **REF)
        print(
            f"  gap {gap_m:.2f} m ({gap} cells): {passable} passable cell(s), "
            f"path {'CROSSES' if crossed else 'does not cross'}"
        )
        if must_pass:
            assert crossed, (f"{gap_m:.2f} m gap walled off - the reference body is "
                             f"{REF_ROBOT_WIDTH_M:.2f} m wide")
        else:
            assert not crossed, f"{gap_m:.2f} m gap accepted - narrower than the body"


def test_default_radii_follow_the_robot_profile() -> None:
    """No radii given (what the planner does on every A*): the field takes the
    clearances of the active profile, read at call time.

    MOSS nominal: lethal 0.22 m, pivot 0.25 m. Overriding MOSS_ROBOT_WIDTH_M
    and MOSS_PIVOT_DIAMETER_M with the reference body's numbers must give the
    reference field, cell for cell, with no restart.
    """
    saved = {k: os.environ.pop(k, None) for k in _PROFILE_ENV}
    try:
        lethal, pivot = clearances()
        d = robot_profile.derived(robot_profile.current())
        print(f"  profile {robot_profile.current().name}: clearances() = ({lethal:.3f}, {pivot:.3f}) m")
        assert (lethal, pivot) == (d.lethal_clearance_m, d.pivot_clearance_m), (lethal, pivot, d)
        assert abs(lethal - MOSS_LETHAL_M) < 1e-9 and abs(pivot - MOSS_PIVOT_M) < 1e-9, (lethal, pivot)

        # the gap table again, under MOSS's own radii: a 0.34 m body needs 0.44 m
        expected = {0.40: (0, False), 0.45: (1, True), 0.50: (2, True),
                    0.55: (3, True), 0.62: (4, True), 0.80: (8, True)}
        for gap_m, (want_cells, must_pass) in expected.items():
            gap, passable, crossed = gap_crossing(gap_m)
            print(f"  MOSS gap {gap_m:.2f} m ({gap} cells): {passable} passable cell(s), "
                  f"path {'CROSSES' if crossed else 'does not cross'}")
            assert passable == want_cells, f"{gap_m:.2f} m: {passable} passable cells, expected {want_cells}"
            assert crossed is must_pass, f"{gap_m:.2f} m: crossed={crossed}, expected {must_pass}"

        os.environ["MOSS_ROBOT_WIDTH_M"] = str(REF_ROBOT_WIDTH_M)
        os.environ["MOSS_PIVOT_DIAMETER_M"] = str(REF_PIVOT_DIAMETER_M)
        lethal, pivot = clearances()
        print(f"  MOSS_ROBOT_WIDTH_M=0.50, MOSS_PIVOT_DIAMETER_M=0.78 -> ({lethal:.3f}, {pivot:.3f}) m")
        assert abs(lethal - REF_LETHAL_CLEARANCE_M) < 1e-9 and abs(pivot - REF_PIVOT_CLEARANCE_M) < 1e-9
        cells = np.zeros((41, 41), dtype=np.int8)
        cells[20, :14] = CostValues.OCCUPIED
        cells[20, 26:] = CostValues.OCCUPIED       # a 0.60 m gap
        grid = make_grid(cells)
        assert np.array_equal(clearance_cost_map(grid).grid, clearance_cost_map(grid, **REF).grid), \
            "the env override must give the reference field exactly"
        print("  env override read at call time: default field == explicit reference field")
    finally:
        for k, v in saved.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v


if __name__ == "__main__":
    for test in (
        test_constants_are_physically_ordered,
        test_prefers_the_wide_corridor,
        test_the_penalty_is_what_chooses_the_wide_corridor,
        test_keeps_away_from_a_lone_wall,
        test_still_takes_a_tight_gap_when_it_is_the_only_way,
        test_default_radii_follow_the_robot_profile,
    ):
        print(test.__name__)
        test()
    print("TEST PASSED")

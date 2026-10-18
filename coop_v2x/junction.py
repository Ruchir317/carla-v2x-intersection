"""Junction geometry: approach sides, turn intents, and which movements conflict.

Everything here uses a right-handed "math frame" (x to the right, y up, counter-clockwise positive).
CARLA's world is left-handed (its y axis points to the right), so positions taken from CARLA are mapped
with `to_math_frame` first. Traffic is assumed to drive on the right, as in CARLA's default towns.

A *movement* is an approach side plus an intent. Side k is the quadrant centred k * 90 degrees
counter-clockwise from the +x axis: 0 = right, 1 = up, 2 = left, 3 = down in the math frame. Each
movement is drawn as a short path through the intersection box, and two movements conflict when their
paths cross or merge (touch). Turns are drawn with a sharp corner, which is deliberately conservative:
opposing left turns are reported as conflicting even though a wide turn could pass between them.
"""
import math
from enum import Enum
from functools import lru_cache
from typing import List, Tuple

Vec = Tuple[float, float]

# Heading of a vehicle approaching from each side (towards the centre).
_HEADINGS: List[Vec] = [(-1.0, 0.0), (0.0, -1.0), (1.0, 0.0), (0.0, 1.0)]

# The same four sides named in CARLA's own axes (the y flip makes math "up" CARLA's -y).
SIDE_NAMES = ("+x", "-y", "-x", "+y")


class Intent(str, Enum):
    LEFT = "left"
    STRAIGHT = "straight"
    RIGHT = "right"


def to_math_frame(x: float, y: float, center_x: float, center_y: float) -> Vec:
    """Position relative to the junction centre, with CARLA's y axis flipped to point up."""
    return (x - center_x, -(y - center_y))


def approach_side(dx: float, dy: float, yaw_deg: float = 0.0) -> int:
    """Which of the four sides a math-frame offset from the centre belongs to (0..3).

    `yaw_deg` rotates the sides for a junction that is not aligned with the axes."""
    angle = math.degrees(math.atan2(dy, dx)) - yaw_deg
    return int(round(angle / 90.0)) % 4


def classify_intent(approach: Vec, depart: Vec, straight_tolerance_deg: float = 45.0) -> Intent:
    """Turn a vehicle makes, from its approach direction and the direction it leaves in (math frame).

    Turns of more than 135 degrees (U-turns) are reported as left or right by sign."""
    cross = approach[0] * depart[1] - approach[1] * depart[0]
    dot = approach[0] * depart[0] + approach[1] * depart[1]
    angle = math.degrees(math.atan2(cross, dot))
    if abs(angle) <= straight_tolerance_deg:
        return Intent.STRAIGHT
    return Intent.LEFT if angle > 0 else Intent.RIGHT


def heading(side: int) -> Vec:
    return _HEADINGS[side % 4]


def exit_heading(side: int, intent: Intent) -> Vec:
    hx, hy = heading(side)
    if intent == Intent.LEFT:
        return (-hy, hx)
    if intent == Intent.RIGHT:
        return (hy, -hx)
    return (hx, hy)


def lane_vector(side: int) -> Vec:
    """Unit vector pointing to the right-hand side of a vehicle approaching from `side`."""
    hx, hy = heading(side)
    return (hy, -hx)


def movement_path(side: int, intent: Intent, half_extent: float, lane_offset: float) -> List[Vec]:
    """Path of a movement through the box, from the entry edge to the exit edge (math frame).

    Vehicles keep to the right: `lane_offset` is the distance of a lane centre from the road axis."""
    hx, hy = heading(side)
    rx, ry = lane_vector(side)
    h, e = half_extent, lane_offset
    local = {  # (distance along the approach heading, distance to its right)
        Intent.STRAIGHT: [(-h, e), (h, e)],
        Intent.LEFT: [(-h, e), (e, e), (e, -h)],
        Intent.RIGHT: [(-h, e), (-e, e), (-e, h)],
    }[intent]
    return [(a * hx + b * rx, a * hy + b * ry) for a, b in local]


def _orientation(p: Vec, q: Vec, r: Vec) -> int:
    value = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return 0 if abs(value) < 1e-9 else (1 if value > 0 else -1)


def _on_segment(p: Vec, q: Vec, r: Vec) -> bool:
    """Is q, known to be collinear with p-r, within the segment p-r?"""
    return min(p[0], r[0]) - 1e-9 <= q[0] <= max(p[0], r[0]) + 1e-9 and min(p[1], r[1]) - 1e-9 <= q[1] <= max(p[1], r[1]) + 1e-9


def segments_intersect(p1: Vec, p2: Vec, p3: Vec, p4: Vec) -> bool:
    """True if the closed segments p1-p2 and p3-p4 cross, touch or overlap."""
    o1, o2 = _orientation(p1, p2, p3), _orientation(p1, p2, p4)
    o3, o4 = _orientation(p3, p4, p1), _orientation(p3, p4, p2)
    if o1 != o2 and o3 != o4:
        return True
    return (
        (o1 == 0 and _on_segment(p1, p3, p2))
        or (o2 == 0 and _on_segment(p1, p4, p2))
        or (o3 == 0 and _on_segment(p3, p1, p4))
        or (o4 == 0 and _on_segment(p3, p2, p4))
    )


def paths_conflict(a: List[Vec], b: List[Vec]) -> bool:
    return any(segments_intersect(a[i], a[i + 1], b[j], b[j + 1]) for i in range(len(a) - 1) for j in range(len(b) - 1))


@lru_cache(maxsize=None)
def movements_conflict(
    side_a: int, intent_a: Intent, side_b: int, intent_b: Intent, half_extent: float = 12.0, lane_offset: float = 3.5
) -> bool:
    """Do two movements cross or merge inside the box? Vehicles on the same side queue in one lane
    behind each other and are never reported as conflicting here (see `movements_block`)."""
    if side_a % 4 == side_b % 4:
        return False
    return paths_conflict(
        movement_path(side_a, intent_a, half_extent, lane_offset),
        movement_path(side_b, intent_b, half_extent, lane_offset),
    )


def movements_block(
    side_a: int, intent_a: Intent, side_b: int, intent_b: Intent, half_extent: float = 12.0, lane_offset: float = 3.5
) -> bool:
    """Must the later of two vehicles wait for the earlier one? True for conflicting movements, and for
    vehicles on the same side (one cannot overtake the other in its lane)."""
    return side_a % 4 == side_b % 4 or movements_conflict(side_a, intent_a, side_b, intent_b, half_extent, lane_offset)

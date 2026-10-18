from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from .junction import Intent, movements_block, movements_conflict
from .messaging import Mailbox, Message, MessageType

POLICIES = ("conflict-aware", "fcfs")

# A waiting vehicle brakes up to a few metres before its stop point, so the approach radius must clear the box.
MIN_APPROACH_MARGIN = 3.0


@dataclass
class VehicleState:
    vehicle_id: int
    arrival_time: float
    side: Optional[int] = None
    intent: Optional[Intent] = None
    x: Optional[float] = None
    y: Optional[float] = None
    last_seen: float = 0.0
    in_box: bool = False
    cleared: bool = False
    permission_time: Optional[float] = None
    enter_time: Optional[float] = None
    exit_time: Optional[float] = None
    exported: bool = False


class IntersectionManager:
    """
    Message-driven intersection manager.

    - A vehicle registers with a REQUEST once it is inside the approach radius, and says where it is going
      with an INTENT (approach side and turn). STATE messages keep its position current.
    - Permissions are granted in arrival order (PERMIT). With the `conflict-aware` policy a vehicle is
      granted only if its movement does not conflict with any vehicle that already holds a permission,
      and it may not overtake an earlier vehicle it conflicts with. With `fcfs` conflicts are ignored and
      `max_active` alone limits how many vehicles hold a permission (default 1: one at a time).
    - When a vehicle enters the intersection box it gets CROSSING; once it has left the box and driven
      clear, its permission is released and it gets CLEARED.
    - A permission the vehicle never used can be revoked after `permission_timeout` seconds, and a vehicle
      that stops reporting for `stale_timeout` seconds is forgotten, so one failure cannot block the queue.
      Revoking frees the manager's slot but does not take the permission back from the vehicle, so set
      the timeout well above the time a vehicle needs to reach the box.

    `center` is anything with `.x` and `.y` in the same frame as the reported positions.
    """

    def __init__(
        self,
        center,
        approach_radius: float = 25.0,
        box_half_extent: float = 8.0,
        policy: str = "conflict-aware",
        max_active: Optional[int] = None,
        permission_timeout: Optional[float] = None,
        stale_timeout: float = 5.0,
        lane_offset: float = 3.5,
    ):
        if policy not in POLICIES:
            raise ValueError(f"policy must be one of {POLICIES}, got {policy!r}")
        if approach_radius < box_half_extent + MIN_APPROACH_MARGIN:
            raise ValueError(
                f"approach radius ({approach_radius:g} m) must be at least {MIN_APPROACH_MARGIN:g} m larger than the "
                f"box half-extent ({box_half_extent:g} m), or vehicles would already be inside the box when they register"
            )
        self.center = center
        self.approach_radius = approach_radius
        self.box_half_extent = box_half_extent
        self.policy = policy
        self.max_active = max_active if max_active is not None else (1 if policy == "fcfs" else None)
        self.permission_timeout = permission_timeout
        self.stale_timeout = stale_timeout
        self.lane_offset = lane_offset
        self._states: Dict[int, VehicleState] = {}
        self._pending_intents: Dict[int, Dict[str, Any]] = {}
        self._active_ids: List[int] = []

    # ------------------------------------------------------------------ one tick

    def step(self, mailbox: Mailbox, timestamp: float) -> None:
        self._ingest(mailbox.drain_manager(), timestamp)
        self._forget_stale(timestamp)
        self._track_crossings(mailbox, timestamp)
        self._revoke_unused(mailbox, timestamp)
        self._grant(mailbox, timestamp)

    def current_permissions(self) -> Dict[int, bool]:
        return {vid: True for vid in self._active_ids}

    def poll_completed(self) -> List[VehicleState]:
        finished: List[VehicleState] = []
        for state in self._states.values():
            if state.cleared and not state.exported:
                finished.append(state)
                state.exported = True
        return finished

    # ------------------------------------------------------------------ stages

    def _ingest(self, messages: List[Message], timestamp: float) -> None:
        for m in messages:
            state = self._states.get(m.vehicle_id)
            if m.type == MessageType.REQUEST and state is None:
                state = VehicleState(vehicle_id=m.vehicle_id, arrival_time=m.timestamp, last_seen=timestamp)
                self._states[m.vehicle_id] = state
                self._apply_intent(state, self._pending_intents.pop(m.vehicle_id, None))
            elif m.type == MessageType.INTENT:
                if state is None:
                    self._pending_intents[m.vehicle_id] = m.payload
                else:
                    self._apply_intent(state, m.payload)
            elif m.type == MessageType.STATE and state is not None:
                state.x, state.y = m.payload["x"], m.payload["y"]
                state.last_seen = timestamp

    @staticmethod
    def _apply_intent(state: VehicleState, payload: Optional[Dict[str, Any]]) -> None:
        if payload:
            state.side = int(payload["side"])
            state.intent = Intent(payload["intent"])

    def _forget_stale(self, timestamp: float) -> None:
        for vid in [v for v, s in self._states.items() if not s.cleared and timestamp - s.last_seen > self.stale_timeout]:
            self._states.pop(vid, None)
            if vid in self._active_ids:
                self._active_ids.remove(vid)

    def _track_crossings(self, mailbox: Mailbox, timestamp: float) -> None:
        for state in self._states.values():
            if state.cleared or state.x is None:
                continue
            in_box = self._is_in_box(state.x, state.y)
            dist = math.hypot(state.x - self.center.x, state.y - self.center.y)
            if in_box and not state.in_box:
                state.in_box = True
                if state.enter_time is None:
                    state.enter_time = timestamp
                self._send(mailbox, MessageType.CROSSING, state, timestamp)
            if state.in_box and not in_box and dist > self.approach_radius * 0.75:
                state.cleared = True
                if state.exit_time is None:
                    state.exit_time = timestamp
                if state.vehicle_id in self._active_ids:
                    self._active_ids.remove(state.vehicle_id)
                self._send(mailbox, MessageType.CLEARED, state, timestamp)

    def _revoke_unused(self, mailbox: Mailbox, timestamp: float) -> None:
        if self.permission_timeout is None:
            return
        for vid in list(self._active_ids):
            state = self._states[vid]
            if (
                state.enter_time is None
                and state.permission_time is not None
                and timestamp - state.permission_time > self.permission_timeout
            ):
                state.cleared = True
                self._active_ids.remove(vid)
                self._send(mailbox, MessageType.CLEARED, state, timestamp)

    def _grant(self, mailbox: Mailbox, timestamp: float) -> None:
        waiting = sorted(
            (s for s in self._states.values() if not s.cleared and s.vehicle_id not in self._active_ids),
            key=lambda s: s.arrival_time,
        )
        active = [self._states[v] for v in self._active_ids]
        for state in self._select(waiting, active, timestamp):
            self._active_ids.append(state.vehicle_id)
            if state.permission_time is None:
                state.permission_time = timestamp
            self._send(mailbox, MessageType.PERMIT, state, timestamp)

    def _select(self, waiting: List[VehicleState], active: List[VehicleState], timestamp: float) -> List[VehicleState]:
        """Which waiting vehicles are granted permission this tick (in arrival order)."""
        granted: List[VehicleState] = []
        skipped: List[VehicleState] = []
        for state in waiting:
            if self.max_active is not None and len(active) + len(granted) >= self.max_active:
                break
            if self.policy == "fcfs":
                granted.append(state)
                continue
            blocked = any(self._conflict(state, other) for other in active + granted) or any(
                self._blocks(earlier, state) for earlier in skipped
            )
            if blocked:
                skipped.append(state)
            else:
                granted.append(state)
        return granted

    # ------------------------------------------------------------------ helpers

    def _conflict(self, a: VehicleState, b: VehicleState) -> bool:
        if a.intent is None or b.intent is None:  # unknown movement: assume the worst
            return True
        return movements_conflict(a.side, a.intent, b.side, b.intent, self.box_half_extent, self.lane_offset)

    def _blocks(self, earlier: VehicleState, later: VehicleState) -> bool:
        if earlier.intent is None or later.intent is None:
            return True
        return movements_block(earlier.side, earlier.intent, later.side, later.intent, self.box_half_extent, self.lane_offset)

    def _is_in_box(self, x: float, y: float) -> bool:
        # Axis-aligned box around the centre. Adjust --box (and the centre) for your target junction.
        return abs(x - self.center.x) <= self.box_half_extent and abs(y - self.center.y) <= self.box_half_extent

    @staticmethod
    def _send(mailbox: Mailbox, type_: MessageType, state: VehicleState, timestamp: float) -> None:
        mailbox.send_to_vehicle(state.vehicle_id, Message(type_, state.vehicle_id, timestamp))


class FixedCycleSignal(IntersectionManager):
    """Baseline: a fixed-cycle traffic light speaking the same protocol, so both can be compared on the
    same scenario. Each phase lasts `green` seconds and is followed by `all_red` seconds in which everything
    is red so the box can empty.

    - `split` (default): the four approaches take turns, one at a time. No two movements ever meet, so
      it is as safe as the conflict-aware manager, which makes it the fair baseline.
    - `axis`: two phases, sides 0 and 2 together then sides 1 and 3. Faster, but turns that cross
      oncoming or merging traffic are not resolved, so it is not conflict-free."""

    def __init__(self, *args, green: float = 10.0, all_red: float = 2.0, phases: str = "split", **kwargs):
        super().__init__(*args, **kwargs)
        if phases not in ("split", "axis"):
            raise ValueError("phases must be 'split' or 'axis'")
        self.green = green
        self.all_red = all_red
        self.phases = phases

    def green_sides(self, timestamp: float) -> Optional[tuple]:
        """The sides that have green right now, or None while everything is red."""
        groups = [(0,), (1,), (2,), (3,)] if self.phases == "split" else [(0, 2), (1, 3)]
        slot = self.green + self.all_red
        position = timestamp % (len(groups) * slot)
        index = int(position // slot)
        return groups[index] if position - index * slot < self.green else None

    def _select(self, waiting: List[VehicleState], active: List[VehicleState], timestamp: float) -> List[VehicleState]:
        sides = self.green_sides(timestamp)
        if sides is None:
            return []
        return [s for s in waiting if s.side in sides]

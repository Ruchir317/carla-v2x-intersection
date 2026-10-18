from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Dict, List, Optional

if TYPE_CHECKING:  # CARLA is only needed for type hints, so the manager can be tested without it
    import carla

    from .vehicle_agent import VehicleAgent


@dataclass
class VehicleState:
    vehicle_id: int
    arrival_time: float
    in_box: bool = False
    cleared: bool = False
    permission_time: Optional[float] = None
    enter_time: Optional[float] = None
    exit_time: Optional[float] = None
    exported: bool = False


class IntersectionManager:
    """
    First-come-first-served intersection manager.
    - Tracks vehicles entering an approach radius.
    - Grants permission in arrival order, up to `max_active` vehicles at a time.
    - Releases a vehicle's permission once it has crossed the intersection box and driven clear.
    - Optionally revokes a permission the vehicle never used (`permission_timeout`, in seconds), so one
      misrouted vehicle cannot block the queue forever.
    """

    def __init__(
        self,
        center: carla.Location,
        approach_radius: float = 25.0,
        box_half_extent: float = 8.0,
        max_active: int = 1,
        permission_timeout: Optional[float] = None,
    ):
        self.center = center
        self.approach_radius = approach_radius
        self.box_half_extent = box_half_extent
        self.permission_timeout = permission_timeout
        self._states: Dict[int, VehicleState] = {}
        self._active_ids: List[int] = []
        self._max_active = max(1, max_active)

    def update(self, agents: List[VehicleAgent], timestamp: float):
        live_ids = {agent.vehicle.id for agent in agents}

        # Register arrivals.
        for agent in agents:
            vid = agent.vehicle.id
            if vid not in self._states and agent.distance_to(self.center) <= self.approach_radius:
                self._states[vid] = VehicleState(vehicle_id=vid, arrival_time=timestamp)

        # Drop removed vehicles from tracking.
        for vid in [vid for vid in self._states if vid not in live_ids]:
            self._states.pop(vid, None)
            if vid in self._active_ids:
                self._active_ids.remove(vid)

        # Update in_box flags and clear vehicles that have crossed and driven away.
        for agent in agents:
            vid = agent.vehicle.id
            state = self._states.get(vid)
            if state is None:
                continue
            in_box = self._is_in_box(agent.vehicle.get_location())
            dist = agent.distance_to(self.center)  # this vehicle's own distance, not the last one seen
            if in_box and not state.in_box:
                state.in_box = True
                if state.enter_time is None:
                    state.enter_time = timestamp
            if state.in_box and not in_box and dist > self.approach_radius * 0.75:
                state.cleared = True
                if state.exit_time is None:
                    state.exit_time = timestamp
                if vid in self._active_ids:
                    self._active_ids.remove(vid)

        # Revoke permissions that were never used.
        if self.permission_timeout is not None:
            for vid in list(self._active_ids):
                state = self._states[vid]
                if (
                    state.enter_time is None
                    and state.permission_time is not None
                    and timestamp - state.permission_time > self.permission_timeout
                ):
                    state.cleared = True
                    self._active_ids.remove(vid)

        # Assign permissions up to the allowed active count.
        waiting = [
            (vid, st.arrival_time)
            for vid, st in self._states.items()
            if not st.cleared and vid not in self._active_ids
        ]
        waiting.sort(key=lambda item: item[1])
        for vid, _ in waiting:
            if len(self._active_ids) >= self._max_active:
                break
            self._active_ids.append(vid)
            state = self._states.get(vid)
            if state and state.permission_time is None:
                state.permission_time = timestamp

    def current_permissions(self) -> Dict[int, bool]:
        return {vid: True for vid in self._active_ids}

    def poll_completed(self) -> List[VehicleState]:
        finished: List[VehicleState] = []
        for state in self._states.values():
            if state.cleared and not state.exported:
                finished.append(state)
                state.exported = True
        return finished

    def _is_in_box(self, location: carla.Location) -> bool:
        # Axis-aligned box around center. Adjust for your target junction geometry if needed.
        return (
            abs(location.x - self.center.x) <= self.box_half_extent
            and abs(location.y - self.center.y) <= self.box_half_extent
        )

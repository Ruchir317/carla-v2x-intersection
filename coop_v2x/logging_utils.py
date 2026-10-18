import csv
from pathlib import Path
from typing import Optional

from .intersection_manager import VehicleState


class MetricsLogger:
    def __init__(self, path: str):
        self.path = Path(path)
        self.file = self.path.open("w", newline="")
        self.writer = csv.DictWriter(
            self.file,
            fieldnames=[
                "vehicle_id",
                "arrival_time",
                "permission_time",
                "enter_time",
                "exit_time",
            ],
        )
        self.writer.writeheader()

    def log_state(self, state: VehicleState):
        self.writer.writerow(
            {
                "vehicle_id": state.vehicle_id,
                "arrival_time": state.arrival_time,
                "permission_time": state.permission_time,
                "enter_time": state.enter_time,
                "exit_time": state.exit_time,
            }
        )
        self.file.flush()

    def close(self):
        self.file.close()

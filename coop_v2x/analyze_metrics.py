"""Summarise a metrics CSV written by run_simulation.py (or demo.py).

    python -m coop_v2x.analyze_metrics metrics.csv

Definitions, per vehicle:
    wait     = permission_time - arrival_time   (time spent queued after reaching the approach radius)
    crossing = exit_time - enter_time           (from entering the box until the vehicle has cleared the junction area)
Overall:
    clearance  = last exit - first arrival      (time to get every vehicle through)
    throughput = vehicles per minute over the clearance window
Vehicles that never entered or left the box (for example a revoked permission) are counted as incomplete.
"""
import argparse
import csv
from typing import Dict, List, Optional

FIELDS = ("arrival_time", "permission_time", "enter_time", "exit_time")


def load_rows(path: str) -> List[Dict[str, Optional[float]]]:
    rows = []
    with open(path, newline="") as f:
        for raw in csv.DictReader(f):
            rows.append({k: (float(raw[k]) if raw.get(k) not in (None, "") else None) for k in FIELDS})
    return rows


def summarise(rows: List[Dict[str, Optional[float]]]) -> Dict[str, float]:
    complete = [r for r in rows if None not in (r["arrival_time"], r["permission_time"], r["enter_time"], r["exit_time"])]
    summary: Dict[str, float] = {"vehicles": len(complete), "incomplete": len(rows) - len(complete)}
    if not complete:
        return summary
    clearance = max(r["exit_time"] for r in complete) - min(r["arrival_time"] for r in complete)
    summary.update(
        avg_wait_s=sum(r["permission_time"] - r["arrival_time"] for r in complete) / len(complete),
        avg_crossing_s=sum(r["exit_time"] - r["enter_time"] for r in complete) / len(complete),
        clearance_s=clearance,
        throughput_per_min=len(complete) / clearance * 60 if clearance > 0 else 0.0,
    )
    return summary


def format_summary(summary: Dict[str, float]) -> str:
    if "avg_wait_s" not in summary:
        return f"No complete vehicle records ({summary['incomplete']} incomplete)."
    return "\n".join(
        [
            f"vehicles:            {summary['vehicles']} complete, {summary['incomplete']} incomplete",
            f"average wait:        {summary['avg_wait_s']:.2f} s",
            f"average crossing:    {summary['avg_crossing_s']:.2f} s",
            f"time to clear all:   {summary['clearance_s']:.2f} s",
            f"throughput:          {summary['throughput_per_min']:.1f} vehicles/min",
        ]
    )


def main():
    parser = argparse.ArgumentParser(description="Summarise an intersection metrics CSV.")
    parser.add_argument("csv_path")
    args = parser.parse_args()
    print(format_summary(summarise(load_rows(args.csv_path))))


if __name__ == "__main__":
    main()

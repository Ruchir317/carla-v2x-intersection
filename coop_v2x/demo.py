"""Run the real controllers against simulated cars. No CARLA needed.

    python -m coop_v2x.demo                       # one run with the conflict-aware manager
    python -m coop_v2x.demo --controller signal   # the fixed-cycle traffic light baseline
    python -m coop_v2x.demo --compare             # all controllers on the same scenario

The cars are a simple kinematic model (see offline_sim.py): good for seeing how each controller orders
and queues traffic, not for predicting CARLA's numbers.
"""
import argparse
import os
import tempfile
from typing import List, Optional, Tuple

from .analyze_metrics import format_summary, load_rows, summarise
from .logging_utils import MetricsLogger
from .messaging import Mailbox
from .offline_sim import Geometry, build_scenario, make_controller, run_scenario

CONTROLLERS = ("conflict-aware", "fcfs", "signal")


def run(
    controller_name: str,
    vehicles: int = 16,
    speed: float = 7.0,
    logfile: Optional[str] = None,
    geometry: Geometry = Geometry(),
    **options,
) -> Tuple[List, dict]:
    """One simulated run. Returns the finished vehicle states and the summary statistics."""
    mailbox = Mailbox()
    controller = make_controller(controller_name, geometry, **options)
    finished = run_scenario(controller, mailbox, build_scenario(vehicles, geometry), geometry, speed=speed)
    if len(finished) < vehicles:  # never average over the cars that happened to finish
        raise RuntimeError(f"only {len(finished)} of {vehicles} vehicles cleared the junction before the time limit")

    path = logfile or tempfile.NamedTemporaryFile(suffix=".csv", delete=False).name
    logger = MetricsLogger(path)
    for state in finished:
        logger.log_state(state)
    logger.close()
    summary = summarise(load_rows(path))
    if logfile is None:
        os.unlink(path)
    return finished, summary


def print_run(finished: List, summary: dict) -> None:
    print("car  side  turn      arrived  allowed  entered  exited")
    for s in sorted(finished, key=lambda s: s.permission_time):
        print(
            f"{s.vehicle_id:>3}  {s.side:>4}  {s.intent.value:<8}  {s.arrival_time:7.1f}  {s.permission_time:7.1f}"
            f"  {s.enter_time:7.1f}  {s.exit_time:6.1f}"
        )
    print()
    print(format_summary(summary))


def print_comparison(vehicles: int, speed: float, geometry: Geometry) -> None:
    rows = [
        ("signal, one approach at a time (conflict-free)", "signal", speed, {"phases": "split"}),
        ("signal, two phases (not conflict-free)", "signal", speed, {"phases": "axis"}),
        ("first-come-first-served, one at a time", "fcfs", speed, {}),
        ("conflict-aware", "conflict-aware", speed, {}),
    ]
    print(
        f"{vehicles} vehicles, approach {geometry.approach:g} m, box half-extent {geometry.box:g} m; "
        f"same scenario in every row\n"
    )
    print(f"{'controller':<48} {'avg wait':>9} {'avg cross':>10} {'clear all':>10} {'veh/min':>8}")
    for label, name, v, options in rows:
        _, s = run(name, vehicles, v, geometry=geometry, **options)
        print(
            f"{label:<48} {s['avg_wait_s']:>8.1f}s {s['avg_crossing_s']:>9.1f}s {s['clearance_s']:>9.1f}s "
            f"{s['throughput_per_min']:>8.1f}"
        )


def main():
    parser = argparse.ArgumentParser(description="Offline demo of the intersection controllers.")
    parser.add_argument("--controller", choices=CONTROLLERS, default="conflict-aware")
    parser.add_argument("--compare", action="store_true", help="run every controller on the same scenario")
    parser.add_argument("--vehicles", type=int, default=16, help="number of cars, spread over four approaches")
    parser.add_argument("--speed", type=float, default=7.0, help="car speed in m/s (7 is about 25 km/h)")
    parser.add_argument("--approach", type=float, default=Geometry.approach, help="approach radius in metres")
    parser.add_argument("--box", type=float, default=Geometry.box, help="half-extent of the intersection box in metres")
    parser.add_argument("--max-active", type=int, default=None, help="cap on cars holding a permission at once")
    parser.add_argument("--green", type=float, default=10.0, help="signal: green seconds per phase")
    parser.add_argument("--all-red", type=float, default=2.0, help="signal: all-red seconds between phases")
    parser.add_argument(
        "--phases", choices=("split", "axis"), default="split", help="signal: one approach at a time, or two axis phases"
    )
    parser.add_argument("--logfile", default=None, help="also write the metrics CSV here")
    args = parser.parse_args()

    geometry = Geometry(approach=args.approach, box=args.box)
    if args.compare:
        print_comparison(args.vehicles, args.speed, geometry)
        return
    if args.controller == "signal":
        options = {"green": args.green, "all_red": args.all_red, "phases": args.phases}
    else:
        options = {"max_active": args.max_active}
    finished, summary = run(args.controller, args.vehicles, args.speed, args.logfile, geometry, **options)
    print_run(finished, summary)
    if args.logfile:
        print(f"\nMetrics written to {args.logfile}")


if __name__ == "__main__":
    main()

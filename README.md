# carla-coop-v2x

Replace a traffic light with a cooperative controller. Vehicles approaching a junction in the [CARLA](https://carla.org/) simulator are queued by an **intersection manager** that lets them through first-come, first-served, and the run writes timing metrics you can analyse.

This is the controller behind the [CARLA cooperative V2X intersection](https://ruchirjadhav.dev/projects/carla-v2x-intersection) project, built for CARLA 0.9.16.

## How it works

1. A vehicle that comes within the **approach radius** of the junction centre registers with the manager.
2. The manager grants **permission** in arrival order, to `--max-active` vehicles at a time (default 1).
3. Without permission a vehicle brakes at its **stop point**, just inside the approach radius. With permission it drives on to its destination.
4. Once a vehicle has crossed the **intersection box** and driven clear, its permission is released and the next vehicle in the queue goes.
5. Arrival, permission, entry and exit times are written to a CSV, and `analyze_metrics` turns that into wait time, crossing time and throughput.

Optionally, `--permission-timeout` revokes a permission the vehicle never used (for example one that was routed away from the junction), so a single misrouted car cannot block everyone behind it.

## Try it without CARLA

The manager's logic runs against simple simulated cars, so you can see the queueing and run the tests on any machine:

```bash
python -m coop_v2x.demo
```

```
car  arrived  allowed  entered  exited
  0      0.8      0.8      4.0     9.5
  1      1.5      9.5     12.8    18.2
  2      2.2     18.2     21.5    26.9
  3      2.9     26.9     30.2    35.7

vehicles:            4 complete, 0 incomplete
average wait:        12.00 s
average crossing:    5.45 s
time to clear all:   34.90 s
throughput:          6.9 vehicles/min
```

Each car is let in only after the previous one has cleared. Try `--max-active 2` to let two through together. This is a logic demonstration with constant-speed cars, not a physics simulation, so its numbers say nothing about the CARLA results.

```bash
pip install -r requirements-dev.txt
pytest
```

The tests cover arrival order, that no two cars share the box, permission release, the timeout, and the metrics maths.

## Run in CARLA

You need a running CARLA 0.9.16 server, the `carla` Python package that matches your CARLA release and Python version, and CARLA's `agents` package. The `agents` package lives in the CARLA install, so point `CARLA_ROOT` at it:

```powershell
$env:CARLA_ROOT = "C:\path\to\CARLA_0.9.16"
python -m coop_v2x.run_simulation --map Town05 `
  --spawn-indices "43,151,177,137" --dest-indices "151,43,137,177" `
  --center-x -45.5 --center-y 4.4 --center-z 0.45 `
  --approach 35 --box 12 --vehicles 4 --logfile metrics.csv
```

Run it from the repository root with `CarlaUE4` already started. On Linux or macOS use `export CARLA_ROOT=...` and drop the backticks.

| Flag | Meaning |
| --- | --- |
| `--map` | Map to load, for example `Town05` |
| `--vehicles` | Number of vehicles to spawn |
| `--spawn-indices`, `--dest-indices` | Comma-separated spawn points and matching destinations, one per vehicle |
| `--center-x/-y/-z` | Junction centre. Without x and y it defaults to the average of the spawn points |
| `--approach` | Approach radius in metres: where vehicles register and queue |
| `--box` | Half-extent of the intersection box in metres |
| `--max-active` | Vehicles allowed through at once (default 1) |
| `--permission-timeout` | Revoke an unused permission after this many simulated seconds (default: never) |
| `--duration` | Stop after this many simulated seconds (default 120) |
| `--logfile` | Metrics CSV path (default `metrics.csv`) |
| `--list-spawns`, `--list-junctions` | Print spawn points or junction centres and exit |

Choose spawn and destination indices that route through your junction centre; otherwise vehicles never enter the managed box.

## Analyse a run

```bash
python -m coop_v2x.analyze_metrics metrics.csv
```

Per vehicle, *wait* is permission time minus arrival time, and *crossing* is the time from entering the box until the vehicle has cleared the junction area. Overall, *time to clear all* runs from the first arrival to the last exit, and *throughput* is vehicles per minute over that window. Vehicles that never completed a crossing are counted as incomplete rather than averaged in.

## Layout

| File | Role |
| --- | --- |
| `coop_v2x/intersection_manager.py` | The first-come-first-served manager (no CARLA dependency) |
| `coop_v2x/vehicle_agent.py` | Wraps CARLA's `BasicAgent`; brakes at the stop point without permission |
| `coop_v2x/run_simulation.py` | CARLA entry point: map, synchronous stepping, spawning, main loop |
| `coop_v2x/logging_utils.py` | CSV metrics writer |
| `coop_v2x/analyze_metrics.py` | Summarises the CSV |
| `coop_v2x/demo.py` | Offline demo with simulated cars |

## Scope and limitations

- This is a deliberately simple controller. It orders vehicles by arrival and does not reason about which movements conflict, so with `--max-active` above 1 it will let vehicles through together even if their paths cross.
- Vehicles are controlled directly from the simulator; there is no modelled radio link, delay or message loss.
- One intersection, with fully cooperative vehicles and no pedestrians or human drivers.
- `run_simulation.py` and `vehicle_agent.py` need a live CARLA server, so the tests exercise the manager and metrics only.

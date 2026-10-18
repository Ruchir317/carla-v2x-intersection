import csv

from coop_v2x.analyze_metrics import format_summary, load_rows, summarise


def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["vehicle_id", "arrival_time", "permission_time", "enter_time", "exit_time"])
        writer.writeheader()
        writer.writerows(rows)


def test_summary_numbers(tmp_path):
    path = tmp_path / "m.csv"
    write_csv(path, [
        {"vehicle_id": 1, "arrival_time": 0, "permission_time": 0, "enter_time": 2, "exit_time": 4},
        {"vehicle_id": 2, "arrival_time": 1, "permission_time": 3, "enter_time": 5, "exit_time": 8},
    ])
    s = summarise(load_rows(str(path)))
    assert s["vehicles"] == 2 and s["incomplete"] == 0
    assert s["avg_wait_s"] == 1.0  # waits of 0 s and 2 s
    assert s["avg_crossing_s"] == 2.5  # crossings of 2 s and 3 s
    assert s["clearance_s"] == 8.0
    assert s["throughput_per_min"] == 15.0  # 2 vehicles in 8 s


def test_incomplete_records_are_counted_not_averaged(tmp_path):
    path = tmp_path / "m.csv"
    write_csv(path, [
        {"vehicle_id": 1, "arrival_time": 0, "permission_time": 0, "enter_time": 1, "exit_time": 3},
        {"vehicle_id": 2, "arrival_time": 0, "permission_time": 5, "enter_time": "", "exit_time": ""},
    ])
    s = summarise(load_rows(str(path)))
    assert s["vehicles"] == 1 and s["incomplete"] == 1
    assert s["avg_crossing_s"] == 2.0


def test_empty_file_reports_no_vehicles(tmp_path):
    path = tmp_path / "m.csv"
    write_csv(path, [])
    s = summarise(load_rows(str(path)))
    assert s == {"vehicles": 0, "incomplete": 0}
    assert "No complete vehicle records" in format_summary(s)

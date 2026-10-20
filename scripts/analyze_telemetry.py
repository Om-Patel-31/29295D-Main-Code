#!/usr/bin/env python3
"""Analyze V5 autonomous telemetry CSV using the Python standard library."""
import argparse
import csv
import math
from collections import defaultdict


def load_rows(path):
    with open(path, newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        required = {"motion", "position_unit", "time_s", "ref_position",
                    "meas_position", "disturb_z3", "axis_cmd_volts"}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError("Missing CSV columns: " + ", ".join(sorted(missing)))
        rows = list(reader)
    if not rows:
        raise ValueError("CSV contains a header but no telemetry rows")
    rows = [row for row in rows if row.get("run_segment", "").strip()]
    if not rows:
        raise ValueError("CSV contains no sample rows")
    return rows


def group_rows(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[(row["motion"], row["position_unit"], row["run_segment"])].append(row)
    return groups


def summarize(groups):
    summaries = []
    for (motion, unit, segment), samples in sorted(groups.items()):
        samples = [r for r in samples if r["ref_position"].strip() and r["meas_position"].strip()]
        if not samples:
            continue
        errors = [float(r["ref_position"]) - float(r["meas_position"]) for r in samples]
        times = [float(r["time_s"]) for r in samples]
        summaries.append({
            "motion": motion, "unit": unit, "segment": segment,
            "samples": len(samples), "duration_s": max(times) - min(times),
            "rmse": math.sqrt(sum(e * e for e in errors) / len(errors)),
            "max_abs_error": max(abs(e) for e in errors),
            "max_abs_disturbance": max(abs(float(r["disturb_z3"])) for r in samples),
        })
    return summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", help="CSV copied from the Brain SD card")
    args = parser.parse_args()
    groups = group_rows(load_rows(args.csv_file))
    summaries = summarize(groups)
    print("motion,unit,segment,samples,duration_s,rmse,max_abs_error,max_abs_disturbance")
    for row in summaries:
        print("{motion},{unit},{segment},{samples},{duration_s:.4f},{rmse:.6f},{max_abs_error:.6f},{max_abs_disturbance:.6f}".format(**row))
    print("\nThese are descriptive controller/odometry summaries, not ground-truth accuracy or baseline comparisons.")

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nOptional plots skipped: install matplotlib to enable plotting.")
        return
    for (motion, unit, segment), samples in sorted(groups.items()):
        samples = [row for row in samples if row["ref_position"].strip() and row["meas_position"].strip()]
        if not samples:
            continue
        time_s = [float(row["time_s"]) for row in samples]
        reference = [float(row["ref_position"]) for row in samples]
        measured = [float(row["meas_position"]) for row in samples]
        errors = [a - b for a, b in zip(reference, measured)]
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        axes[0].plot(time_s, reference, "k--", label="Reference")
        axes[0].plot(time_s, measured, "b-", label="Measured")
        axes[0].set_ylabel("Position ({})".format(unit)); axes[0].legend(); axes[0].grid(True)
        axes[1].plot(time_s, errors, "r-"); axes[1].set_ylabel("Error ({})".format(unit)); axes[1].grid(True)
        axes[2].plot(time_s, [float(r["disturb_z3"]) for r in samples], "g-", label="ESO z3")
        axes[2].plot(time_s, [float(r["axis_cmd_volts"]) for r in samples], "m--", label="Axis command (V)")
        axes[2].set_xlabel("Time (s)"); axes[2].legend(); axes[2].grid(True)
        if all(row.get("path_ref_x_in", "").strip() and row.get("pose_x_in", "").strip() for row in samples):
            fig2, ax = plt.subplots(figsize=(7, 7))
            ax.plot([float(r["path_ref_x_in"]) for r in samples],
                    [float(r["path_ref_y_in"]) for r in samples], "k--", label="Path reference")
            ax.plot([float(r["pose_x_in"]) for r in samples],
                    [float(r["pose_y_in"]) for r in samples], "b-", label="Odometry")
            ax.set_aspect("equal", adjustable="datalim"); ax.set_xlabel("X (in)"); ax.set_ylabel("Y (in)")
            ax.legend(); ax.grid(True); ax.set_title("Path segment {}".format(segment))
            fig2.tight_layout(); fig2.savefig("path_segment_{}.png".format(segment), dpi=200)
            plt.close(fig2)
        fig.suptitle("{} segment {}".format(motion, segment))
        fig.tight_layout()
        fig.savefig("telemetry_{}_segment_{}.png".format(motion, segment), dpi=200)
        plt.close(fig)


if __name__ == "__main__":
    main()

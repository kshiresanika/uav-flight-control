#!/usr/bin/env python3
"""Plot a telemetry CSV written by fc-app."""
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import Patch

MODE_COLORS = {
    "Degraded": "#f4b400",
    "Failsafe": "#db4437",
    "Disarmed": "#7f7f7f",
}


def mode_spans(df):
    """Yield (start_time, end_time, mode) for each run of identical modes."""
    modes = df["mode"].to_numpy()
    times = df["time_s"].to_numpy()
    start = 0
    for i in range(1, len(modes) + 1):
        if i == len(modes) or modes[i] != modes[start]:
            end = times[i] if i < len(modes) else times[-1]
            yield times[start], end, modes[start]
            start = i


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: python scripts/plot_telemetry.py <telemetry.csv> [output.png]")
        return 1
    csv_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else csv_path.with_suffix(".png")

    df = pd.read_csv(csv_path)  # "NaN" in the file becomes a real NaN
    t = df["time_s"]

    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)

    ax = axes[0]
    ax.plot(t, np.degrees(df["roll_true"]), label="true")
    ax.plot(t, np.degrees(df["roll_est"]), "--", label="estimate")
    ax.plot(t, np.degrees(df["roll_set"]), ":", label="setpoint")
    ax.set_ylabel("roll [deg]")
    ax.legend(loc="upper right")

    ax = axes[1]
    ax.plot(t, np.degrees(df["pitch_true"]), label="pitch true")
    ax.plot(t, np.degrees(df["pitch_est"]), "--", label="pitch estimate")
    ax.plot(t, np.degrees(df["yaw_true"]), label="yaw true")
    ax.set_ylabel("pitch / yaw [deg]")
    ax.legend(loc="upper right")

    ax = axes[2]
    ax.plot(t, np.degrees(df["gyro_x"]), label="gyro x (raw)")
    faults = df[df["gyro_fault"] == 1]
    if not faults.empty:
        ax.scatter(
            faults["time_s"],
            np.zeros(len(faults)),
            marker="|",
            color="red",
            label="gyro fault flag",
        )
    ax.set_ylabel("roll rate [deg/s]")
    ax.legend(loc="upper right")

    ax = axes[3]
    for name in ("m1", "m2", "m3", "m4"):
        ax.plot(t, df[name], label=name.upper())
    ax.set_ylim(-0.05, 1.05)
    ax.set_ylabel("motor command")
    ax.set_xlabel("time [s]")
    ax.legend(loc="upper right", ncol=4)

    # Shade every non-Normal flight mode on all panels.
    present = set()
    for start, end, mode in mode_spans(df):
        color = MODE_COLORS.get(mode)
        if color is None:
            continue
        present.add(mode)
        for a in axes:
            a.axvspan(start, end, color=color, alpha=0.15, linewidth=0)

    for a in axes:
        a.grid(True, alpha=0.3)

    if present:
        handles = [
            Patch(color=MODE_COLORS[m], alpha=0.3, label=m)
            for m in MODE_COLORS
            if m in present
        ]
        fig.legend(handles=handles, loc="upper center", ncol=len(handles))

    fig.suptitle(csv_path.stem, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_path, dpi=150)
    print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
#!/usr/bin/env python3
"""
Plot step response data captured over UART.

Usage:
  1. Connect to the Nucleo VCP: screen /dev/tty.usbmodem* 115200
  2. Run: step 30
  3. Copy CSV output to a file (or pipe: minicom -C log.csv)
  4. python3 plot_step_response.py log.csv

CSV format: time_ms, setpoint, measurement, output
"""

import sys
import csv
import matplotlib.pyplot as plt
import numpy as np


def load_data(path):
    times, setpoints, measurements, outputs = [], [], [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) != 4:
                continue
            try:
                times.append(float(parts[0]))
                setpoints.append(float(parts[1]))
                measurements.append(float(parts[2]))
                outputs.append(float(parts[3]))
            except ValueError:
                continue
    return (
        np.array(times),
        np.array(setpoints),
        np.array(measurements),
        np.array(outputs),
    )


def analyze(times, setpoints, measurements):
    if len(times) < 2:
        return {}

    sp = setpoints[0]
    step_size = abs(sp - measurements[0])
    if step_size < 0.1:
        return {}

    final_val = measurements[-1]
    peak_val = np.max(np.abs(measurements))

    # Rise time: 10% to 90% of step
    target_10 = measurements[0] + 0.1 * step_size * np.sign(sp - measurements[0])
    target_90 = measurements[0] + 0.9 * step_size * np.sign(sp - measurements[0])

    t_10, t_90 = None, None
    for i, m in enumerate(measurements):
        if t_10 is None and abs(m - measurements[0]) >= abs(target_10 - measurements[0]):
            t_10 = times[i]
        if t_90 is None and abs(m - measurements[0]) >= abs(target_90 - measurements[0]):
            t_90 = times[i]

    rise_time = (t_90 - t_10) if (t_10 is not None and t_90 is not None) else None

    # Overshoot
    overshoot = 0
    if step_size > 0:
        overshoot = (peak_val - abs(sp)) / step_size * 100

    # Settling time (2% band)
    band = 0.02 * step_size
    settle_time = None
    for i in range(len(times) - 1, -1, -1):
        if abs(measurements[i] - sp) > band:
            if i + 1 < len(times):
                settle_time = times[i + 1]
            break

    # Steady-state error
    ss_error = abs(sp - final_val)

    return {
        "rise_time_ms": rise_time,
        "overshoot_pct": overshoot,
        "settling_time_ms": settle_time,
        "steady_state_error": ss_error,
    }


def main():
    if len(sys.argv) < 2:
        print(f"usage: {sys.argv[0]} <csv_file>")
        sys.exit(1)

    times, setpoints, measurements, outputs = load_data(sys.argv[1])
    if len(times) == 0:
        print("no data found")
        sys.exit(1)

    stats = analyze(times, setpoints, measurements)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax1.plot(times, setpoints, "r--", label="Setpoint", linewidth=1.5)
    ax1.plot(times, measurements, "b-", label="Measurement", linewidth=1)
    ax1.set_ylabel("Angle (deg)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    info = []
    if stats.get("rise_time_ms") is not None:
        info.append(f"Rise: {stats['rise_time_ms']:.0f} ms")
    if stats.get("overshoot_pct") is not None:
        info.append(f"OS: {stats['overshoot_pct']:.1f}%")
    if stats.get("settling_time_ms") is not None:
        info.append(f"Settle: {stats['settling_time_ms']:.0f} ms")
    info.append(f"SSE: {stats.get('steady_state_error', 0):.2f} deg")
    ax1.set_title("Step Response  |  " + "  |  ".join(info))

    ax2.plot(times, outputs, "g-", label="Control Effort", linewidth=1)
    ax2.set_ylabel("Output (normalized)")
    ax2.set_xlabel("Time (ms)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim(-1.1, 1.1)

    plt.tight_layout()
    plt.savefig("step_response.png", dpi=150)
    plt.show()

    print("\nStep Response Metrics:")
    for k, v in stats.items():
        print(f"  {k}: {v}")


if __name__ == "__main__":
    main()

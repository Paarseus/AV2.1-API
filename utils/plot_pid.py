#!/usr/bin/env python3
"""
Plot PID tuning results.

Usage:
    python plot_pid.py pid_tune_20241126_143022.csv
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt


def main():
    if len(sys.argv) < 2:
        print("Usage: python plot_pid.py <csv_file>")
        return 1

    df = pd.read_csv(sys.argv[1])

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    # Speed
    axes[0].plot(df['time'], df['target'], 'r--', label='Target', lw=2)
    axes[0].plot(df['time'], df['actual'], 'b-', label='Actual', lw=2)
    axes[0].set_ylabel('Speed (m/s)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # Error
    axes[1].plot(df['time'], df['error'], 'g-', lw=2)
    axes[1].axhline(0, color='k', ls='--', alpha=0.5)
    axes[1].set_ylabel('Error (m/s)')
    axes[1].grid(True, alpha=0.3)

    # Throttle
    axes[2].plot(df['time'], df['throttle'], 'm-', lw=2)
    axes[2].set_ylabel('Throttle')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylim(0, 1)
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(sys.argv[1].replace('.csv', '.png'), dpi=150)
    plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())

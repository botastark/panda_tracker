#!/usr/bin/env python3
"""
Plot tracker pose variation while the robot is stationary.

Usage:
    python3 plot_tracker_static_noise.py diagnostics_axis.csv

Outputs:
    tracker_position_variation.png
    tracker_orientation_variation.png

The script keeps only rows where tracker_sequence_changed == 1, so each
tracker packet is plotted once even if the observer loop logs it multiple times.
"""

import argparse
import pandas as pd
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", help="diagnostics_axis.csv")
    args = parser.parse_args()

    df = pd.read_csv(args.csv)

    # Keep valid, newly received tracker packets only.
    d = df[
        (df["tracker_valid"] == 1)
        & (df["tracker_sequence_changed"] == 1)
    ].copy()

    if d.empty:
        raise RuntimeError("No valid new tracker packets found in the CSV.")

    # Time relative to first retained tracker packet.
    t = d["elapsed_s"] - d["elapsed_s"].iloc[0]

    # ------------------------------------------------------------
    # 1. RAW tracker translation T_CT
    #    Plot variation around the median, in millimetres.
    # ------------------------------------------------------------
    pos_cols = ["raw_CT_x_m", "raw_CT_y_m", "raw_CT_z_m"]
    pos = d[pos_cols]
    pos_median = pos.median()
    pos_delta_mm = (pos - pos_median) * 1000.0

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(t, pos_delta_mm["raw_CT_x_m"], label="x")
    ax.plot(t, pos_delta_mm["raw_CT_y_m"], label="y")
    ax.plot(t, pos_delta_mm["raw_CT_z_m"], label="z")
    ax.axhline(0.0, linewidth=1)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Variation from median [mm]")
    ax.set_title("Raw tracker T_CT position variation — stationary robot")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig("tracker_position_variation.png", dpi=160)

    # ------------------------------------------------------------
    # 2. RAW tracker orientation T_CT
    # ------------------------------------------------------------
    ori_cols = [
        "raw_CT_roll_deg",
        "raw_CT_pitch_deg",
        "raw_CT_yaw_deg",
    ]
    ori = d[ori_cols]

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(t, ori["raw_CT_roll_deg"], label="roll")
    ax.plot(t, ori["raw_CT_pitch_deg"], label="pitch")
    ax.plot(t, ori["raw_CT_yaw_deg"], label="yaw")
    ax.axhline(0.0, linewidth=1)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angle [deg]")
    ax.set_title("Raw tracker T_CT orientation — stationary robot")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig("tracker_orientation_variation.png", dpi=160)

    # ------------------------------------------------------------
    # Simple numeric summary
    # ------------------------------------------------------------
    print(f"Tracker packets plotted: {len(d)}")
    print()
    print("RAW T_CT position median [m]:")
    print(pos_median.to_string())

    print()
    print("RAW T_CT position standard deviation [mm]:")
    print((pos.std() * 1000.0).to_string())

    print()
    print("RAW T_CT position peak-to-peak [mm]:")
    print(((pos.max() - pos.min()) * 1000.0).to_string())

    print()
    print("RAW T_CT orientation median [deg]:")
    print(ori.median().to_string())

    print()
    print("RAW T_CT orientation standard deviation [deg]:")
    print(ori.std().to_string())

    print()
    print("RAW T_CT orientation peak-to-peak [deg]:")
    print((ori.max() - ori.min()).to_string())

    # ------------------------------------------------------------
    # 3. Stationary statistics in one figure with two separate plots:
    #    top    -> position variables in mm
    #    bottom -> orientation variables in deg
    #
    # Each plot shows Mean, Std, and Peak-to-peak.
    # ------------------------------------------------------------
    pos_mm = pos * 1000.0

    pos_mean = pos_mm.mean()
    ori_mean = ori.mean()

    pos_std = pos_mm.std()
    ori_std = ori.std()

    pos_ptp = pos_mm.max() - pos_mm.min()
    ori_ptp = ori.max() - ori.min()

    pos_labels = ["X", "Y", "Z"]
    ori_labels = ["Roll", "Pitch", "Yaw"]

    pos_mean_values = [
        pos_mean["raw_CT_x_m"],
        pos_mean["raw_CT_y_m"],
        pos_mean["raw_CT_z_m"],
    ]
    pos_std_values = [
        pos_std["raw_CT_x_m"],
        pos_std["raw_CT_y_m"],
        pos_std["raw_CT_z_m"],
    ]
    pos_ptp_values = [
        pos_ptp["raw_CT_x_m"],
        pos_ptp["raw_CT_y_m"],
        pos_ptp["raw_CT_z_m"],
    ]

    ori_mean_values = [
        ori_mean["raw_CT_roll_deg"],
        ori_mean["raw_CT_pitch_deg"],
        ori_mean["raw_CT_yaw_deg"],
    ]
    ori_std_values = [
        ori_std["raw_CT_roll_deg"],
        ori_std["raw_CT_pitch_deg"],
        ori_std["raw_CT_yaw_deg"],
    ]
    ori_ptp_values = [
        ori_ptp["raw_CT_roll_deg"],
        ori_ptp["raw_CT_pitch_deg"],
        ori_ptp["raw_CT_yaw_deg"],
    ]

    width = 0.24

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 9))

    x_pos = range(len(pos_labels))
    ax1.bar(
        [i - width for i in x_pos],
        pos_mean_values,
        width,
        label="Mean",
    )
    ax1.bar(
        list(x_pos),
        pos_std_values,
        width,
        label="Std",
    )
    ax1.bar(
        [i + width for i in x_pos],
        pos_ptp_values,
        width,
        label="Peak-to-peak",
    )
    ax1.set_xticks(list(x_pos))
    ax1.set_xticklabels(pos_labels)
    ax1.set_ylabel("Position [mm]")
    ax1.set_title("Raw tracker T_CT position statistics")
    ax1.grid(True, axis="y", alpha=0.3)
    ax1.legend()

    x_ori = range(len(ori_labels))
    ax2.bar(
        [i - width for i in x_ori],
        ori_mean_values,
        width,
        label="Mean",
    )
    ax2.bar(
        list(x_ori),
        ori_std_values,
        width,
        label="Std",
    )
    ax2.bar(
        [i + width for i in x_ori],
        ori_ptp_values,
        width,
        label="Peak-to-peak",
    )
    ax2.set_xticks(list(x_ori))
    ax2.set_xticklabels(ori_labels)
    ax2.set_ylabel("Orientation [deg]")
    ax2.set_title("Raw tracker T_CT orientation statistics")
    ax2.grid(True, axis="y", alpha=0.3)
    ax2.legend()

    fig.suptitle("Stationary tracker variation summary")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig("tracker_variable_statistics_split.png", dpi=160)

    print()
    print("Position statistics [mm]:")
    for label, mean_v, std_v, ptp_v in zip(
        pos_labels, pos_mean_values, pos_std_values, pos_ptp_values
    ):
        print(
            f"  {label:5s} mean={mean_v:.6f} "
            f"std={std_v:.6f} "
            f"peak-to-peak={ptp_v:.6f}"
        )

    print()
    print("Orientation statistics [deg]:")
    for label, mean_v, std_v, ptp_v in zip(
        ori_labels, ori_mean_values, ori_std_values, ori_ptp_values
    ):
        print(
            f"  {label:5s} mean={mean_v:.6f} "
            f"std={std_v:.6f} "
            f"peak-to-peak={ptp_v:.6f}"
        )

    print()
    print("Saved:")
    print("  tracker_position_variation.png")
    print("  tracker_orientation_variation.png")
    print("  tracker_variable_statistics_split.png")

    plt.show()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Live plots for CSV emitted by pbvs_robot_position_telemetry.cpp."""

import argparse
import csv
import os
import sys
from collections import deque

try:
    import numpy as np
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtWidgets
except ImportError as exc:
    raise SystemExit(
        "Missing plotting packages. Install with: "
        "python3 -m pip install numpy pyqtgraph PyQt5"
    ) from exc


class CsvTail:
    def __init__(self, path: str, max_rows: int):
        self.path = path
        self.rows = deque(maxlen=max_rows)
        self.stream = None
        self.header = None
        self.position = 0

    def _close(self):
        if self.stream is not None:
            self.stream.close()
        self.stream = None
        self.header = None
        self.position = 0

    def poll(self):
        if not os.path.exists(self.path):
            return
        if self.stream is not None and os.path.getsize(self.path) < self.position:
            self._close()
            self.rows.clear()
        if self.stream is None:
            self.stream = open(self.path, "r", newline="", encoding="utf-8")
            header_line = self.stream.readline()
            if not header_line.endswith("\n"):
                self._close()
                return
            self.header = next(csv.reader([header_line]))
            self.position = self.stream.tell()

        while True:
            line_start = self.stream.tell()
            line = self.stream.readline()
            if not line:
                self.stream.seek(line_start)
                break
            if not line.endswith("\n"):
                self.stream.seek(line_start)
                break
            try:
                values = next(csv.reader([line]))
                if len(values) != len(self.header):
                    continue
                self.rows.append(dict(zip(self.header, map(float, values))))
            except (ValueError, csv.Error):
                continue
            self.position = self.stream.tell()


class TelemetryWindow(QtWidgets.QMainWindow):
    COLORS = {"x": "#ef5350", "y": "#66bb6a", "z": "#42a5f5"}

    def __init__(self, args):
        super().__init__()
        self.args = args
        self.tail = CsvTail(args.csv, max_rows=max(2000, int(args.window * 200)))
        self.setWindowTitle("Panda PBVS telemetry")
        self.resize(1550, 950)

        pg.setConfigOptions(antialias=False, background="#111318", foreground="#d7dae0")
        self.graphics = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.graphics)
        self.dash_line = getattr(
            getattr(QtCore.Qt, "PenStyle", QtCore.Qt), "DashLine"
        )

        self.plots = {}
        self.curves = {}
        self._make_plot("target", "Tracker translation (camera frame)", "mm", 0, 0)
        self._make_plot("error", "Active tracking error (base frame)", "mm", 0, 1)
        self._make_plot("velocity", "Velocity stages (base frame)", "mm/s", 1, 0)
        self._make_plot("force", "Bias-subtracted external force", "N", 1, 1)
        self._make_plot("torque", "Bias-subtracted external torque", "Nm", 2, 0)
        self._make_plot("health", "Timing and safety", "value", 2, 1)

        for axis in "xyz":
            color = self.COLORS[axis]
            self._curve("target", f"raw_tracker_{axis}_mm", f"raw {axis}", color, "DashLine")
            self._curve("target", f"filtered_tracker_{axis}_mm", f"filtered {axis}", color)
            self._curve("error", f"error_{axis}_mm", axis, color)
            self._curve("velocity", f"servo_v{axis}_mmps", f"servo {axis}", color, "DashLine")
            self._curve("velocity", f"achieved_v{axis}_mmps", f"achieved {axis}", color)

        for axis in "xyz":
            self._curve("force", f"f{axis}_n", f"F{axis}", self.COLORS[axis])
            self._curve("torque", f"t{axis}_nm", f"T{axis}", self.COLORS[axis])
        self._curve("force", "force_norm_n", "|F|", "#ffffff", width=2)
        self._curve("torque", "torque_norm_nm", "|T|", "#ffffff", width=2)
        self._curve("health", "tracker_age_ms", "tracker age (ms)", "#ffa726")
        self._curve("health", "safety_scale", "safety scale", "#ab47bc", width=2)

        self.plots["force"].addItem(
            pg.InfiniteLine(args.force_limit, angle=0, pen=pg.mkPen("#ff1744", width=2)))
        self.plots["torque"].addItem(
            pg.InfiniteLine(args.torque_limit, angle=0, pen=pg.mkPen("#ff1744", width=2)))
        self.plots["torque"].addItem(
            pg.InfiniteLine(
                args.torque_limit * args.derate_ratio,
                angle=0,
                pen=pg.mkPen("#ffb300", width=1, style=self.dash_line),
            )
        )

        for name, plot in self.plots.items():
            if name != "target":
                plot.setXLink(self.plots["target"])

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(50)

    def _make_plot(self, key, title, units, row, col):
        plot = self.graphics.addPlot(row=row, col=col, title=title)
        plot.showGrid(x=True, y=True, alpha=0.25)
        plot.setLabel("left", units)
        plot.setLabel("bottom", "time", "s")
        plot.addLegend(offset=(8, 8), labelTextSize="8pt")
        self.plots[key] = plot

    def _curve(self, plot_key, column, label, color, style=None, width=1):
        qt_style = getattr(
            getattr(QtCore.Qt, "PenStyle", QtCore.Qt), "SolidLine"
        )
        if style == "DashLine":
            qt_style = self.dash_line
        self.curves[column] = self.plots[plot_key].plot(
            name=label, pen=pg.mkPen(color, width=width, style=qt_style)
        )

    def refresh(self):
        self.tail.poll()
        if not self.tail.rows:
            self.statusBar().showMessage(f"Waiting for {self.args.csv}")
            return

        rows = list(self.tail.rows)
        newest = rows[-1]["time_s"]
        cutoff = newest - self.args.window
        rows = [row for row in rows if row["time_s"] >= cutoff]
        time = np.fromiter((row["time_s"] for row in rows), dtype=float)

        for column, curve in self.curves.items():
            if column not in rows[-1]:
                continue
            values = np.fromiter((row[column] for row in rows), dtype=float)
            values[~np.isfinite(values)] = np.nan
            curve.setData(time, values)

        last = rows[-1]
        self.statusBar().showMessage(
            f"t={newest:.2f}s   |e|="
            f"{np.linalg.norm([last['error_x_mm'], last['error_y_mm'], last['error_z_mm']]):.2f} mm   "
            f"|F|={last['force_norm_n']:.2f} N   |T|={last['torque_norm_nm']:.3f} Nm   "
            f"safety={last['safety_scale']:.2f}   armed={int(last['armed'])}"
        )
        self.plots["target"].setXRange(max(0.0, cutoff), newest, padding=0.01)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", help="CSV path passed to --telemetry-csv")
    parser.add_argument("--window", type=float, default=15.0, help="visible seconds")
    parser.add_argument("--force-limit", type=float, default=15.0)
    parser.add_argument("--torque-limit", type=float, default=3.0)
    parser.add_argument("--derate-ratio", type=float, default=0.8)
    return parser.parse_args()


def main():
    args = parse_args()
    app = QtWidgets.QApplication(sys.argv)
    window = TelemetryWindow(args)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

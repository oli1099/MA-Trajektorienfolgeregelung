#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CreateTrajectory.py – Erzeugung und Visualisierung einer Spline-Trajektorie
für einen mobilen Roboter (ohne ROS2-Part).
"""

from __future__ import annotations
import argparse
import csv
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

# -----------------------------------------------------------------------------
# Konfigurationsparameter
# -----------------------------------------------------------------------------
# Waypoints im Weltkoordinatensystem [m]
DEFAULT_WAYPOINTS: np.ndarray = np.array([
    (0.00, 0.00, 0),
    (0.25, 0.00, 0),
    (0.50, 0.00, 0),
    (0.75, 0.20, 0),
    (0.90, 0.36, 0),
    (1.25, 0.40, 0),
    (1.50, 0.40, 0),
    (1.75, 0.40, 0),
    (2.00, 0.38, 0),
    (2.25, 0.20, 0),
    (2.50, 0.00, 0),
    (2.75, 0.00, 0),
    (3.00, 0.00, 0),
], dtype=float)

V_MAX: float = 0.16              # Soll-Geschwindigkeit [m/s]
DT_SAMPLE: float = 0.1           # Zeitauflösung für die Abtastung [s]

# Farbpalette der TU Chemnitz
PALETTE: list[str] = [
    "#009D81",  # Basisfarbe
    "#33B19A",  # 20% Tint
    "#66C4B3",  # 40% Tint
    "#99D8CD",  # 60% Tint
    "#CCEBE6",  # 80% Tint
]


def build_splines(
    waypoints: np.ndarray,
    bc_type: str = 'clamped'
) -> tuple[np.ndarray, CubicSpline, CubicSpline]:
    """
    Erzeugt kubische Splines x(s), y(s) parametrisiert nach Bogenlänge s.
    """
    # Segmentlängen zwischen aufeinanderfolgenden Wegpunkten
    segment_lengths = np.linalg.norm(
        np.diff(waypoints[:, :2], axis=0), axis=1
    )
    # Bogenlängen-Knotenpunkte
    s = np.insert(np.cumsum(segment_lengths), 0, 0.0)
    # Splines für x- und y-Koordinate
    spline_x = CubicSpline(s, waypoints[:, 0], bc_type=bc_type)
    spline_y = CubicSpline(s, waypoints[:, 1], bc_type=bc_type)
    return s, spline_x, spline_y


def write_csv(
    filename: Path | str,
    t: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    yaw: np.ndarray
) -> None:
    """
    Speichert Zeit, Position und Orientierung als CSV-Datei.
    """
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t [s]', 'x [m]', 'y [m]', 'yaw [rad]'])
        for row in zip(t, x, y, yaw):
            writer.writerow(map(float, row))
    print(f'CSV gespeichert: {filename}')


def plot_trajectory(
    x: np.ndarray,
    y: np.ndarray,
    waypoints: np.ndarray
) -> None:
    """
    Zeichnet die Trajektorie und Wegpunkte in einem 2D-Plot.
    """
    fig, ax = plt.subplots(figsize=(7.29, 2.3))
    # Trajektorie
    ax.plot(
        x, y,
        linestyle='-',
        linewidth=1,
        color=PALETTE[0],
        label='Trajektorie'
    )
    # Wegpunkte
    ax.scatter(
        waypoints[:, 0], waypoints[:, 1],
        color=PALETTE[1],
        s=30,
        zorder=5,
        label='Wegpunkte'
    )
    # Achsenbeschriftung
    ax.set_xlabel(r'$x$ in $\mathrm{m}$', fontsize=11)
    ax.set_ylabel(r'$y$ in $\mathrm{m}$', fontsize=11)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)
    ax.set_ylim(bottom=-0.1, top=0.5)

    # Legende oberhalb der Figure zentriert
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(
        handles, labels,
        loc='upper center',
        bbox_to_anchor=(0.5, 0.98),
        ncol=len(labels),
        fontsize='small',
        frameon=True
    )

    # Layout anpassen und speichern
    plt.tight_layout(rect=[0, 0, 1, 0.91])
    plt.savefig('trajectory.svg', format='svg')
    plt.show()


def main() -> None:
    """
    Hauptfunktion: Parsen der Argumente, Berechnung der Splines,
    Sampling der Trajektorie und Ausgabe (CSV oder Plot).
    """
    parser = argparse.ArgumentParser(
        description='Spline-Trajektorie erzeugen und plotten'
    )
    parser.add_argument(
        '--csv', metavar='FILE',
        help='Pfad zur CSV-Datei, falls Ausgabe gewünscht'
    )
    parser.add_argument(
        '--speed', type=float, default=V_MAX,
        help=f'Soll-Geschwindigkeit [m/s] (default {V_MAX})'
    )
    args = parser.parse_args()

    # Splines erzeugen
    s, sx, sy = build_splines(DEFAULT_WAYPOINTS)

    # Zeit-Parameter und s-Gitter erzeugen
    s_end = s[-1]
    t_end = s_end / args.speed
    t = np.arange(0.0, t_end + DT_SAMPLE, DT_SAMPLE)
    s_grid = args.speed * t

    # Trajektorie und Ableitungen auswerten
    x = sx(s_grid)
    y = np.maximum(sy(s_grid), 0.0)
    dx = sx(s_grid, 1)
    dy = sy(s_grid, 1)
    yaw = np.unwrap(np.arctan2(dy, dx))
    yaw = np.maximum(yaw, 0.0)

    # CSV-Ausgabe, falls gewünscht
    if args.csv:
        write_csv(args.csv, t, x, y, yaw)
    else:
        # Plotten, wenn keine CSV-Ausgabe
        plot_trajectory(x, y, DEFAULT_WAYPOINTS)


if __name__ == '__main__':
    main()

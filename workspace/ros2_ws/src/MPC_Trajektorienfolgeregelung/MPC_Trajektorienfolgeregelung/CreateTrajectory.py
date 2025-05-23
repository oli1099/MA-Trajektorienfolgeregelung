#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
trajectory_gen.py  –  Spline-Trajektorie für mobilen Roboter erzeugen und plotten
"""
from __future__ import annotations
import argparse, csv, math, sys, time
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# TU CI Farbpalette
my_palette = [
    "#009D81",  # Base
    "#33B19A",  # 20% Tint
    "#66C4B3",  # 40% Tint
    "#99D8CD",  # 60% Tint
    "#CCEBE6",  # 80% Tint
]

# -------------------------- Parameter -------------------------------------

# ► Way-Points im Welt-Koordinatensystem [m]
DEFAULT_WPS = np.asarray([
    (0.00, 0.00, 0),
    (0.25, 0.00, 0),
    (0.5, 0, 0),
    (0.75, 0.2, 0),
    (0.9, 0.36, 0),
    (1.25, 0.40, 0),
    (1.5, 0.40, 0),
    (1.75, 0.4, 0),
    (2, 0.38, 0),
    (2.25, 0.2, 0),
    (2.5, 0, 0),
    (2.75, 0, 0),
    (3, 0, 0)
], dtype=float)

RESOLUTION      = 0.02     # [m] Abtastabstand entlang des Pfades
V_MAX           = 0.16     # [m/s] Soll-Geschwindigkeit fürs einfache Zeitgesetz
BC_TYPE         = 'clamped'  # Randbedingungen: 'natural'| 'clamped'
ROS_TOPIC       = '/ref_path'

# --------------------------------------------------------------------------

def build_splines(waypoints: np.ndarray, bc: str = 'clamped'):
    """Erzeuge Cubic-Splines x(s), y(s) mit Parameter s = Bogenlänge."""
    seg_len = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    s = np.insert(np.cumsum(seg_len), 0, 0.0)
    sx = CubicSpline(s, waypoints[:, 0], bc_type=bc)
    sy = CubicSpline(s, waypoints[:, 1], bc_type=bc)
    return s, sx, sy


def write_csv(fname: Path | str, t, x, y, yaw):
    with open(fname, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t [s]', 'x [m]', 'y [m]', 'yaw [rad]'])
        for row in zip(t, x, y, yaw):
            w.writerow(map(float, row))
    print(f'CSV gespeichert: {fname}')


def main():
    ap = argparse.ArgumentParser(description='Spline-Trajektorie erzeugen und plotten')
    ap.add_argument('--csv', metavar='FILE', help='Trajektorie als CSV sichern')
    ap.add_argument('--ros', action='store_true', help='in ROS 2 publizieren')
    ap.add_argument('--speed', type=float, default=V_MAX,
                    help=f'Soll-v_max [m/s] (default {V_MAX})')
    args = ap.parse_args()

    # 1  Splines
    s, sx, sy = build_splines(DEFAULT_WPS, BC_TYPE)

    # 2+3  Gleichmäßiges Zeit-Sampling dt = 0.1 s
    s_end = s[-1]
    t_end = s_end / args.speed
    dt = 0.1
    t = np.arange(0.0, t_end + dt, dt)
    s_grid = args.speed * t
    x = sx(s_grid)
    y = np.maximum(sy(s_grid), 0.0)
    dx = sx(s_grid, 1)
    dy = sy(s_grid, 1)
    yaw = np.unwrap(np.arctan2(dy, dx))
    yaw = np.maximum(yaw, 0.0)

    # 4  CSV?
    if args.csv:
        write_csv(args.csv, t, x, y, yaw)

    # 5  ROS 2?
    if args.ros:
        try:
            import rclpy
            from rclpy.node import Node
            from nav_msgs.msg import Path
            from geometry_msgs.msg import PoseStamped, Quaternion
            from tf_transformations import quaternion_from_euler

            rclpy.init()
            node = rclpy.create_node('traj_pub')
            pub = node.create_publisher(Path, ROS_TOPIC, 10)

            # Publish-Funktion
            def publish_path(t, x, y, yaw):
                msg = Path()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                for xi, yi, psi in zip(x, y, yaw):
                    pose = PoseStamped()
                    pose.header = msg.header
                    pose.pose.position.x = float(xi)
                    pose.pose.position.y = float(yi)
                    q = quaternion_from_euler(0.0, 0.0, float(psi))
                    pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                    msg.poses.append(pose)
                pub.publish(msg)
                node.get_logger().info(f'Published Path with {len(msg.poses)} poses')

            publish_path(t, x, y, yaw)
            print('Ctrl-C zum Beenden …')
            rclpy.spin(node)
        except ModuleNotFoundError as e:
            sys.exit(f'ROS 2 Pakete fehlen: {e}')
    else:
        # Plot der Trajektorie und Waypoints
        fig, ax = plt.subplots(figsize=(7.29, 2))
        ax.plot(x, y,
                linestyle='-', linewidth=1,
                color=my_palette[0], label='Trajektorie')
        ax.scatter(DEFAULT_WPS[:,0], DEFAULT_WPS[:,1],
                   color=my_palette[1], s=30, zorder=5,
                   label='Waypoints')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_aspect('equal', adjustable='box')
        #ax.set_title('Spline-Trajektorie mit Waypoints')
        ax.legend(fontsize='small')
        ax.grid(True)
        #ax.axis('equal')
        ax.set_ylim(bottom=-0.1, top=0.5)  # Y-Achse bis -0.1 begrenzen
        xlim_top = ax.get_xlim()[1]
        
        #ax.set_xlim(top = 0.5)  # Y-Achse bis -0.1 begrenzen
        
        
        plt.show()


if __name__ == '__main__':
    main()

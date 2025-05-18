#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
trajectory_gen.py  –  Spline-Trajektorie für mobilen Roboter erzeugen

Aufrufbeispiele
---------------
# Nur CSV ausgeben
python3 trajectory_gen.py --csv traj.csv

# Live in ROS 2 (Topic /ref_path) + CSV
python3 trajectory_gen.py --ros --csv traj.csv

Abhängigkeiten
--------------
numpy, scipy, (optional) rclpy, geometry_msgs, nav_msgs
"""
from __future__ import annotations
import argparse, csv, math, sys, time
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline

# -------------------------- Parameter -------------------------------------

# ► Way-Points im Welt-Koordinatensystem [m]
DEFAULT_WPS = np.asarray( [(0.00, 0.00, 0),   # Index 0
    (0.25, 0.00, 0),   # Index 4
    (0.5, 0, 0),
    (0.75, 0.2,0),
       #(0.9,0.36,0),   # Index 8
    (0.9, 0.36, 0),   # Index 13
    (1.25, 0.40, 0),   # Index 17
    (1.5, 0.40, 0),   # Index 21
    (1.75, 0.4, 0),   # Index 25
    (2, 0.38, 0),   # Index 30
    (2.25, 0.2, 0),   # Index 34
    (2.5, 0, 0),
     (2.75,0,0),
      (3,0,0) ], dtype=float)

RESOLUTION      = 0.02     # [m] Abtastabstand entlang des Pfades
V_MAX           = 0.16    # [m/s] Soll-Geschwindigkeit fürs einfache Zeitgesetz
BC_TYPE         = 'clamped'  # Rand­bedingungen: 'natural'| 'clamped'

ROS_TOPIC       = '/ref_path'

# --------------------------------------------------------------------------


def build_splines(waypoints: np.ndarray, bc: str = 'clamped'):
    """Erzeuge Cubic-Splines x(s), y(s) mit Parameter s = Bogenlänge."""
    # s-Koordinate (kumulierte Sehnenlänge)
    seg_len = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    s = np.insert(np.cumsum(seg_len), 0, 0.0)
    sx = CubicSpline(s, waypoints[:, 0], bc_type=bc)
    sy = CubicSpline(s, waypoints[:, 1], bc_type=bc)
    return s, sx, sy


def sample_path(s_end: float, sx: CubicSpline, sy: CubicSpline,
                ds: float = 0.02):
    """Gleichmäßig abtasten, Orientierung als Yaw berechnen."""
    s_grid = np.arange(0, s_end + ds, ds)
    x  = sx(s_grid)
    y  = sy(s_grid)
    dx = sx(s_grid, 1)
    dy = sy(s_grid, 1)
    yaw = np.unwrap(np.arctan2(dy, dx))
    return s_grid, x, y, yaw


def time_parameter(s_grid: np.ndarray, v_max: float):
    """Simple lineares Zeitgesetz: t = s / v_max."""
    return s_grid / v_max


# ----------------------------- ROS 2 --------------------------------------

def ros_publisher_init(node_name: str = 'traj_pub'):
    """ROS 2 initialisieren und Path-Publisher zurückgeben."""
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped, Quaternion
    from tf_transformations import quaternion_from_euler

    rclpy.init()
    node: Node = rclpy.create_node(node_name)
    pub = node.create_publisher(Path, ROS_TOPIC, 10)

    def publish_path(t, x, y, yaw):
        msg = Path()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for ti, xi, yi, psi in zip(t, x, y, yaw):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(xi)
            pose.pose.position.y = float(yi)
            q: tuple[float, float, float, float] = quaternion_from_euler(
                0.0, 0.0, float(psi))
            pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            msg.poses.append(pose)
        pub.publish(msg)
        node.get_logger().info(f'Published Path with {len(msg.poses)} poses')

    return rclpy, publish_path


# ----------------------------- CSV ----------------------------------------

def write_csv(fname: Path | str, t, x, y, yaw):
    with open(fname, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t [s]', 'x [m]', 'y [m]', 'yaw [rad]'])
        for row in zip(t, x, y, yaw):
            w.writerow(map(float, row))
    print(f'CSV gespeichert: {fname}')


# -------------------------------- main ------------------------------------

def main():
    ap = argparse.ArgumentParser(description='Spline-Trajektorie erzeugen')
    ap.add_argument('--csv', metavar='FILE', help='Trajektorie als CSV sichern')
    ap.add_argument('--ros', action='store_true', help='in ROS 2 publizieren')
    ap.add_argument('--speed', type=float, default=V_MAX,
                    help=f'Soll-v_max [m/s] (default {V_MAX})')
    args = ap.parse_args()

    # 1  Splines
    s, sx, sy = build_splines(DEFAULT_WPS, BC_TYPE)

    # 2  Abtasten
    #s_grid, x, y, yaw = sample_path(s[-1], sx, sy, RESOLUTION)
    # alle negativen y auf 0 setzen
    #y = np.maximum(y, 0.0)
    #yaw = np.maximum(yaw, 0.0)


    # 3  Zeitgesetz
    #t = time_parameter(s_grid, args.speed)

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
            rclpy, pub_fn = ros_publisher_init()
            pub_fn(t, x, y, yaw)
            print('Ctrl-C zum Beenden …')
            rclpy.spin(rclpy.get_default_context().get_nodes()[0])
        except ModuleNotFoundError as e:
            sys.exit(f'ROS 2 Pakete fehlen: {e}')
    else:
        # Minimaler Matplotlib-Plot, falls gewünscht
        try:
            import matplotlib.pyplot as plt
            plt.figure()
            plt.plot(x, y, '-', label='Spline')
            plt.scatter(*DEFAULT_WPS.T, c='r', label='Way-Points')
            plt.axis('equal'); plt.legend(); plt.grid(True)
            plt.title('Spline-Pfad')
            plt.show()
        except ModuleNotFoundError:
            pass


if __name__ == '__main__':
    main()

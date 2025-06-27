#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
trajectory_pcontroller.py – ROS2-Node für trajektorienfolgende Regelung
mit Proportional-Controller und MPC für ein Mecanum-Fahrgestell.
"""

from __future__ import annotations
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import MotorsState

from controller.mecanum import MecanumChassis
from MPC.SystemModel import DynamicModel
from MPC.SaveData import SaveData
import matplotlib.pyplot as plt

# -----------------------------------------------------------------------------
# Konfigurationsparameter
# -----------------------------------------------------------------------------
TRAJECTORY_CSV: Path = Path(
    '/home/prinzessinleia/PrinzessinLeia/RepoTrajektorienfolgeregelung'
    '/workspace/ros2_ws/src/MPC_Trajektorienfolgeregelung'
    '/MPC_Trajektorienfolgeregelung/traj.csv'
)
LOOKAHEAD_DISTANCE: float = 0.01  # Look-ahead-Abstand [m]
APPROACH_SPEED_MAX: float = 0.1   # maximaler Approach-Speed [m/s]
V_REF: float = 0.1               # Soll-Geschwindigkeit [m/s]
TOLERANCE: float = 0.05          # Wegpunkt-Toleranz [m]
P_GAIN_LAT: float = 0.5          # Querfehler-Gain
P_GAIN_PSI: float = 1.0         # Heading-Gain

class TrajectoryPController(Node):  # pylint: disable=too-many-instance-attributes
    """
    ROS2-Node für die Trajektorienverfolgung.
    Liest Wegpunkte aus einer CSV, berechnet P-Steuerung und MPC
    und publiziert Rad- und Befehlsgeschwindigkeiten.
    """

    def __init__(self) -> None:
        super().__init__('trajectory_pcontroller')

        # Trajektorie aus CSV laden
        data = np.loadtxt(TRAJECTORY_CSV, delimiter=',', skiprows=1)
        # Daten: [t, x, y, yaw]
        self.trajectory: list[tuple[float, float, float]] = [
            (row[1], row[2], row[3]) for row in data
        ]

        # Steuerungsparameter
        self.lp = LOOKAHEAD_DISTANCE
        self.ua_max = APPROACH_SPEED_MAX
        self.v_ref = V_REF
        self.tolerance = TOLERANCE
        self.k_lat = P_GAIN_LAT
        self.k_psi = P_GAIN_PSI

        # Initialisierung
        self.start_time: rclpy.time.Time | None = None
        self.current_position: tuple[float, float] | None = None
        self.current_orientation: float = 0.0

        # Datenlogging
        self.actual_path: list[tuple[tuple[float, float], float]] = []
        self.actual_u: list[np.ndarray] = []
        self.solve_times: list[float] = []

        # Modelle
        self.mecanum = MecanumChassis()
        self.mpc = DynamicModel()

        # ROS2 Publisher & Subscriber
        self.motor_pub = self.create_publisher(
            MotorsState, 'ros_robot_controller/set_motor', 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer für Regel- und Plotschleife
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.plot_timer = self.create_timer(1.0, self.plot_callback)

        # Echtzeit-Plot aktivieren
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.fig_u, self.ax_u = plt.subplots()

    def odom_callback(self, msg: Odometry) -> None:
        """
        Verarbeitet Odometry-Updates, speichert Pose und Startzeit.
        """
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        self.current_orientation = self._quaternion_to_yaw(
            msg.pose.pose.orientation
        )
        if self.start_time is None:
            self.start_time = self.get_clock().now()

    @staticmethod
    def _quaternion_to_yaw(q) -> float:
        """
        Konvertiert eine Quaternion in den Yaw-Winkel.
        """
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def compute_reference(self) -> tuple[float, float]:
        """
        Bestimmt den Look-ahead-Punkt auf der Trajektorie und liefert
        den LOS-Fehlervektor.
        """
        if self.current_position is None:
            return 0.0, 0.0

        # Index des nächsten Wegpunkts
        p = np.array(self.current_position)
        dists = [np.linalg.norm(p - np.array(pt[:2])) for pt in self.trajectory]
        i_min = int(np.argmin(dists))

        # Akkumulierte Distanz entlang der Trajektorie
        dist_acc = 0.0
        idx = i_min
        while idx < len(self.trajectory) - 1 and dist_acc < self.lp:
            p0 = np.array(self.trajectory[idx][:2])
            p1 = np.array(self.trajectory[idx + 1][:2])
            dist_acc += np.linalg.norm(p1 - p0)
            idx += 1

        x_la, y_la, _ = self.trajectory[idx]
        ex = x_la - self.current_position[0]
        ey = y_la - self.current_position[1]
        return ex, ey

    def control_loop(self) -> None:
        """
        Regelschleife: Berechnet Fehler, publiziert Befehle.
        """
        if self.current_position is None or self.start_time is None:
            return

        # Relative Zeit
        elapsed = self.get_clock().now() - self.start_time
        t_rel = elapsed.nanoseconds * 1e-9
        self.actual_path.append((self.current_position, t_rel))

        # LOS-Fehler
        ex, ey = self.compute_reference()

        # Normierung mit Approach-Speed
        denom = math.hypot(ex, ey) + self.lp
        vx = self.ua_max * (ex / denom)
        vy = self.ua_max * (ey / denom)

        # Heading-Regler
        phi_d = math.atan2(vy, vx)
        err_phi = math.atan2(
            math.sin(phi_d - self.current_orientation),
            math.cos(phi_d - self.current_orientation)
        )
        theta = self.k_psi * err_phi

        # MPC-Berechnung der Radraten
        omega_vec = self.mpc.get_omega(vx, vy, 0)
        omega_vec = np.clip(omega_vec, -5.0, 5.0)

        # Motorbefehle
        motor_msg = self.mecanum.set_velocity(vx, vy, theta)
        self.motor_pub.publish(motor_msg)

        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(theta)
        self.cmd_pub.publish(twist)

        # Logging
        self.actual_u.append(omega_vec)
        self.get_logger().info(
            f"e=({ex:.3f},{ey:.3f}), vx={vx:.3f}, vy={vy:.3f}",
            throttle_duration_sec=1.0
        )

        # Abbruch bei letzter Wegpunkt-Erreichung
        if math.hypot(ex, ey) < self.tolerance and idx == len(self.trajectory) - 1:
            self.stop_robot()

    def stop_robot(self) -> None:
        """
        Stoppt den Roboter, speichert Plots und Log-Daten.
        """
        # Anhalten
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        self.motor_pub.publish(self.mecanum.set_velocity(0, 0, 0))

        # Plots speichern
        self.fig.savefig("trajectory_plot.png")
        self.fig_u.savefig("u_plot.png")

        # Daten speichern
        sd = SaveData(
            [],
            self.actual_path,
            self.actual_u,
            [],
            [],
            self.solve_times
        )
        sd.save_all("mpc_data")

        # Timer abbrechen
        self.control_timer.cancel()
        self.plot_timer.cancel()

    def plot_callback(self) -> None:
        """
        Visualisiert Soll- und Ist-Trajektorie sowie Radgeschwindigkeiten in Echtzeit.
        """
        if not self.actual_path:
            return

        # Solltrajektorie
        xs, ys, _ = zip(*self.trajectory)
        self.ax.cla()
        self.ax.plot(xs, ys, 'r--', label='Solltrajektorie')

        # Ist-Pfad
        path_x = [p[0] for p, _ in self.actual_path]
        path_y = [p[1] for p, _ in self.actual_path]
        self.ax.plot(path_x, path_y, 'b-', label='Ist-Pfad')
        self.ax.set_aspect('equal', 'datalim')
        self.ax.set_xlabel('x [m]')
        self.ax.set_ylabel('y [m]')
        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        # Radraten
        u_arr = np.array(self.actual_u)
        self.ax_u.cla()
        if u_arr.size:
            for i in range(u_arr.shape[1]):
                self.ax_u.plot(u_arr[:, i], label=f'Rad {i+1}')
            self.ax_u.set_xlabel('Iteration')
            self.ax_u.set_ylabel('Winkelgeschw. [rad/s]')
            self.ax_u.grid(True)
            self.ax_u.legend()
        self.fig_u.canvas.draw()
        self.fig_u.canvas.flush_events()


def main(args=None) -> None:
    """
    Einstiegspunkt: Initialisiert den ROS2-Node und startet das Spin-Loop.
    """
    rclpy.init(args=args)
    node = TrajectoryPController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

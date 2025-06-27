#!/usr/bin/env python3
import math
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
from pathlib import Path

class TrajectoryPController(Node):
    def __init__(self):
        super().__init__('trajectory_pcontroller')
        # Trajektorie laden aus CSV
        csv_file = Path('/home/prinzessinleia/PrinzessinLeia/RepoTrajektorienfolgeregelung/MA-Trajektorienfolgeregelung/workspace/ros2_ws/src/MPC_Trajektorienfolgeregelung/MPC_Trajektorienfolgeregelung/traj.csv')
        data = np.loadtxt(csv_file, delimiter=',', skiprows=1)
        xs   = data[:,1]
        ys   = data[:,2]
        yaws = data[:,3]
        self.trajectory = list(zip(xs, ys, yaws))

        # Regelparameter
        self.Lp      = 0.1   # Look-ahead Abstand
        self.Vref    = 0.2     # Sollgeschwindigkeit
        self.epsilon = 0.05    # Toleranz für via-Points
        self.Ua_max  = 0.1     # max. Approach-Speed (nicht genutzt hier)
        self.k_psi   = 1.0     # Heading-Gain

        # Zustand
        self.start_timer = None
        self.current_position = None
        self.current_orientation = 0.0

        # Data Logging
        self.actual_path = []
        self.actual_u = []
        self.predictions_list = []
        self.actual_theta = []
        self.predicted_theta_list = []
        self.solve_times = []

        # Mecanum und MPC
        self.mecanum_chassis = MecanumChassis()
        self.mpc_model = DynamicModel()

        # ROS-Publisher und Subscriber
        self.motor_pub = self.create_publisher(MotorsState, 'ros_robot_controller/set_motor', 10)
        self.get_position = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)
        self.plot_timer = self.create_timer(1.0, self.plot_callback)

        # Plot initialisieren
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.fig_u, self.ax_u = plt.subplots()

    def odom_callback(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        self.current_orientation = self.quaternion_to_yaw(msg.pose.pose.orientation)
        if self.start_timer is None:
            self.start_timer = self.get_clock().now()

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def is_via_point(self, x_ref, y_ref, yaw_ref):
        # Jeder diskretisierte Trajekt-Punkt ist via-Point
        return True

    def compute_reference(self):
        """
        Finde den ersten Trajektorie-Punkt jenseits der Toleranz epsilon.
        Berechne dann Vref_x und Vref_y gemäß Algorithmus 1.
        """
        x_hat, y_hat = self.current_position
        Vref_x = 0.0
        Vref_y = 0.0

        for idx, (xr, yr, yaw) in enumerate(self.trajectory):
            if idx % 5 != 0:
                continue
            dx = xr - x_hat
            dy = yr - y_hat
            D  = math.hypot(dx, dy)

            # Jeder Punkt ist via-Point: nur epsilon-Prüfung
            if D <= self.epsilon:
                continue

            # Gefunden: normierter Vektor mit Sättigung
            K_V = max(0.0, min(D / self.Lp, 1.0))
            if D > 0:
                dir_x = dx / D
                dir_y = dy / D
                Vref_x = self.Vref * K_V * np.cos(np.arctan2(dir_y, dir_x))
                Vref_y = self.Vref * K_V * np.sin(np.arctan2(dir_y, dir_x))
            break

        return Vref_x, Vref_y

    def control_loop(self):
        if self.current_position is None:
            return

        now = self.get_clock().now()
        delta = now - self.start_timer
        t_rel = delta.nanoseconds * 1e-9
        self.actual_path.append((self.current_position, t_rel))

        # Berechne Referenz-Geschwindigkeit
        v_x, v_y = self.compute_reference()

        # Heading-Regler
        phi_d = math.atan2(v_y, v_x)
        err_phi = math.atan2(
            math.sin(phi_d - self.current_orientation),
            math.cos(phi_d - self.current_orientation)
        )
        theta = self.k_psi * err_phi

        # MPC Stellgrößen
        omega_vec = self.mpc_model.get_omega(v_x, v_y, 0)
        omega_vec = np.clip(omega_vec, -5.0, 5.0)

        # Motorbefehle
        motor_msg = self.mecanum_chassis.set_velocity(v_x, v_y, theta)
        self.motor_pub.publish(motor_msg)

        twist = Twist()
        twist.linear.x = float(v_x)
        twist.linear.y = float(v_y)
        twist.angular.z = float(theta)
        self.cmd_pub.publish(twist)

        # Log
        self.actual_u.append(omega_vec)
        self.get_logger().info(
            f"v_ref=({v_x:.3f},{v_y:.3f}), theta={theta:.3f}, pos={self.current_position}"
        )

    def stop_robot(self):
        # Stoppbefehle
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.linear.y = 0.0
        stop_twist.angular.z = 0.0
        self.cmd_pub.publish(stop_twist)

        stop_motor = self.mecanum_chassis.set_velocity(0, 0, 0)
        self.motor_pub.publish(stop_motor)

        # Daten speichern und Timer stoppen
        self.fig.savefig("trajectory_plot.png")
        self.fig_u.savefig("u_plot.png")
        sd = SaveData(
            self.predictions_list,
            self.actual_path,
            self.actual_u,
            self.actual_theta,
            self.predicted_theta_list,
            self.solve_times
        )
        sd.save_all("mpc_data")
        self.timer.cancel()

    def plot_callback(self):
        if not self.actual_path:
            return
        # Solltrajektorie
        traj_x = [p[0] for p in self.trajectory]
        traj_y = [p[1] for p in self.trajectory]
        self.ax.cla()
        self.ax.plot(traj_x, traj_y, 'r--', label='Solltrajektorie')
        # Tatsächlicher Pfad
        path_x = [p[0][0] for p in self.actual_path]
        path_y = [p[0][1] for p in self.actual_path]
        self.ax.plot(path_x, path_y, 'b-', label='Ist-Pfad')
        self.ax.set_aspect('equal', 'datalim')
        self.ax.set_xlabel('x [m]')
        self.ax.set_ylabel('y [m]')
        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        # Stellgrößen-Plots
        self.ax_u.cla()
        u_arr = np.array(self.actual_u)
        if u_arr.size > 0:
            t = np.arange(u_arr.shape[0])
            for i in range(u_arr.shape[1]):
                self.ax_u.plot(t, u_arr[:, i], label=f'Rad {i+1}')
            self.ax_u.set_xlabel('Iterationen')
            self.ax_u.set_ylabel('Winkelgeschw. [rad/s]')
            self.ax_u.legend()
            self.ax_u.grid(True)
        self.fig_u.canvas.draw()
        self.fig_u.canvas.flush_events()


def main(args=None):
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
